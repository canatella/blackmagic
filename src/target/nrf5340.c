#include <stdlib.h>
#include <inttypes.h>

#include "gdb_packet.h"
#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "adiv5.h"
#include "cortex.h"
#include "cortexm.h"

#define NRF5340_DESIGNER                     0x244U
#define NRF5340_PARTNO                       0x70U
#define NRF5340_RESET_BASE                   0x50005000U
#define NRF5340_NETWORK_FORCE_OFF            (NRF5340_RESET_BASE + 0x614)
#define NRF5340_NETWORK_FORCE_OFF_WORKAROUND (NRF5340_RESET_BASE + 0x618)

#define FICR_INFO_RAM           0x218U
#define FICR_INFO_CODEPAGE_SIZE 0x220U
#define FICR_INFO_CODE_SIZE     0x224U

#define UICR_AP_PROTECT        0x0U
#define UICR_SECURE_AP_PROTECT 0x01CU

#define NVMC_READY     0x400
#define NVMC_CONFIG    0x504
#define NVMC_ERASE_ALL 0x50C

#define NRF53_CTRL_AP_IDR              0x12880000U
#define CTRL_AP_POWER_EN               ADIV5_DP_REG(0x01U)
#define CTRL_AP_SELECT_AP              ADIV5_DP_REG(0x02U)
#define CTRL_AP_PROTECT_STATUS         ADIV5_AP_REG(0x0CU)
#define CTRL_AP_PROTECT_DISABLE        ADIV5_AP_REG(0x010U)
#define CTRL_SECURE_AP_PROTECT_DISABLE ADIV5_AP_REG(0x014U)
#define CTRL_ERASE_ALL                 ADIV5_AP_REG(0x04U)
#define CTRL_ERASE_ALL_STATUS          ADIV5_AP_REG(0x08U)
#define CTRL_ERASE_PROTECT_STATUS      ADIV5_AP_REG(0x01C)

#define DEB(...)           \
	gdb_outf(__VA_ARGS__); \
	DEBUG_ERROR(__VA_ARGS__);

typedef enum core_type {
	core_app = 0,
	core_net = 1
} core_type_e;

typedef enum ap_type {
	ap_ahb_app,
	ap_ahb_net,
	ap_ctrl_app,
	ap_ctrl_net
} ap_type_e;

struct layout {
	const uint32_t ram_base;
	const uint32_t flash_base;
	const uint32_t ficr_base;
	const uint32_t uicr_base;
	const uint32_t nvmc_base;
	const uint32_t codepage_size;
	const uint32_t code_size;
	const char *driver;
};
typedef struct layout layout_s;

static const layout_s layouts[] = {[core_app] = {.flash_base = 0x00000000U,
									   .ram_base = 0x20000000U,
									   .ficr_base = 0x00FF0000U,
									   .uicr_base = 0x00FF8000U,
									   .nvmc_base = 0x50039000U,
									   .codepage_size = 0x1000U,
									   .code_size = 0x100U,
									   .driver = "nRF53 App AHB AP"},
	[core_net] = {.flash_base = 0x01000000U,
		.ram_base = 0x21000000U,
		.ficr_base = 0x01FF0000U,
		.uicr_base = 0x01FF8000U,
		.nvmc_base = 0x41080000U,
		.codepage_size = 0x800U,
		.code_size = 0x80U,
		.driver = "nRF53 Net AHB AP"}};

struct data {
	const layout_s *layout;
	adiv5_access_port_s *ctrl_ap;
};

typedef struct data data_s;

static bool recover = false;

static void data_free(data_s *data)
{
	if (data->ctrl_ap != NULL) {
		adiv5_ap_unref(data->ctrl_ap);
		data->ctrl_ap = NULL;
	}

	free(data);
}

static inline const data_s *target_data(target_s *target)
{
	return target->target_storage;
}

static inline uint32_t ficr_read(target_s *target, uint32_t offset)
{
	return target_mem32_read32(target, target_data(target)->layout->ficr_base + offset);
}

static inline uint32_t uicr_read(target_s *target, uint32_t offset)
{
	return target_mem32_read32(target, target_data(target)->layout->uicr_base + offset);
}

enum nvmc_configs {
	nvmc_cfg_read_only_access = 0,
	nvmc_cfg_write_enabled = 1,
	nvmc_cfg_erase_enabled = 2,
	nvmc_cfg_partial_erase_enabled = 3
};

static inline uint32_t nvmc_read(target_s *target, uint32_t offset)
{
	return target_mem32_read32(target, target_data(target)->layout->nvmc_base + offset);
}

static inline bool nvmc_write(target_s *target, uint32_t offset, uint32_t value)
{
	target_mem32_write32(target, target_data(target)->layout->nvmc_base + offset, value);
	while (nvmc_read(target, NVMC_READY) == 0) {
		if (target_check_error(target))
			return false;
	}
	return true;
}

static bool nrf5340_flash_erase(target_flash_s *flash, target_addr_t addr, size_t length);
static bool nrf5340_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t length);
static bool nrf5340_flash_prepare(target_flash_s *flash);
static bool nrf5340_flash_done(target_flash_s *flash);
static void recover_if_protected(adiv5_debug_port_s *dp);

static void nrf5340_add_flash(target_s *target, uint32_t addr, size_t length, size_t erasesize)
{
	target_flash_s *flash = calloc(1, sizeof(*flash));
	if (!flash) {
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return;
	}

	flash->start = addr;
	flash->length = length;
	flash->blocksize = erasesize;
	/* Limit the write buffer size to 1k to help prevent probe memory exhaustion */
	flash->writesize = MIN(erasesize, 1024U);
	flash->erase = nrf5340_flash_erase;
	flash->write = nrf5340_flash_write;
	flash->prepare = nrf5340_flash_prepare;
	flash->done = nrf5340_flash_done;
	flash->erased = 0xff;
	target_add_flash(target, flash);
}

static bool nrf5340_wait_ready(target_s *const target, platform_timeout_s *const print_progress)
{
	/* Poll for NVMC_READY */
	while (nvmc_read(target, NVMC_READY) == 0) {
		if (target_check_error(target))
			return false;

		if (print_progress)
			target_print_progress(print_progress);
	}

	return true;
}

static inline bool is_app(const layout_s *layout)
{
	return layout->flash_base == 0;
}

static inline bool is_net(const layout_s *layout)
{
	return !is_app(layout);
}

static bool nrf5340_flash_prepare(target_flash_s *const flash)
{
	target_s *const target = flash->t;

	switch (flash->operation) {
	case FLASH_OPERATION_WRITE:
		return nvmc_write(target, NVMC_CONFIG, nvmc_cfg_write_enabled);
	case FLASH_OPERATION_ERASE:
		return nvmc_write(target, NVMC_CONFIG, nvmc_cfg_erase_enabled);
	default:
		DEBUG_INFO("unknown flash operation %u\n", flash->operation);
		return false; /* Unsupported operation */
	}
}

static bool nrf5340_flash_done(target_flash_s *flash)
{
	target_s *target = flash->t;

	/* Return to read-only */
	return nvmc_write(target, nvmc_cfg_read_only_access, nvmc_cfg_read_only_access);
}

static bool nrf5340_flash_erase(target_flash_s *flash, target_addr_t addr, size_t len)
{
	target_s *target = flash->t;

	for (size_t offset = 0; offset < len; offset += flash->blocksize) {
		/* Write all ones to first word in page to erase it */
		target_mem32_write32(target, addr + offset, 0xffffffffU);

		if (!nrf5340_wait_ready(target, NULL))
			return false;
	}

	return true;
}

static bool nrf5340_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t len)
{
	/* nrf5340_flash_prepare() and nrf5340_flash_done() top-and-tail this, just write the data to the target. */
	target_s *target = flash->t;
	target_mem32_write(target, dest, src, len);
	return nrf5340_wait_ready(target, NULL);
}

static bool nrf5340_mass_erase(target_s *const target, platform_timeout_s *const print_progess)
{
	bool status = false;

	/* Enable erase */
	if (!nvmc_write(target, NVMC_CONFIG, nvmc_cfg_erase_enabled))
		goto error;

	/* Erase all */
	if (!nvmc_write(target, NVMC_ERASE_ALL, 1U))
		goto error_read_only;

	status = nrf5340_wait_ready(target, print_progess);

error_read_only:
	status = status && nvmc_write(target, NVMC_CONFIG, nvmc_cfg_read_only_access);
error:
	return status;
}

static void nrf5340_release_network_core(target_s *target)
{
	if (!is_app(target_data(target)->layout)) {
		DEBUG_ERROR("trying to start nrf5340 network core from bad ap");
		return;
	}

	// Try to power on network core
	// https://github.com/zephyrproject-rtos/hal_nordic/blob/427ee1a519e8a0844d0f78f7cbc8cdfc134ef00d/nrfx/hal/nrf_reset.h#L175
	target_mem32_write32(target, NRF5340_NETWORK_FORCE_OFF_WORKAROUND, 1);
	target_mem32_write32(target, NRF5340_NETWORK_FORCE_OFF, 0);

	platform_delay(1);
	target_mem32_write32(target, NRF5340_NETWORK_FORCE_OFF, 1);

	platform_delay(1);
	target_mem32_write32(target, NRF5340_NETWORK_FORCE_OFF, 0);
	target_mem32_write32(target, NRF5340_NETWORK_FORCE_OFF_WORKAROUND, 0);
	platform_delay(1);
}

static void nrf5340_extended_reset(target_s *target)
{
	if (is_app(target_data(target)->layout)) {
		// Errata 97
		if (target_mem32_read32(target, 0x50004A20U) == 0U) {
			target_mem32_write32(target, 0x50004A20, 0xDU);
			target_mem32_write32(target, 0x5000491C + 0, 0x1U);
			target_mem32_write32(target, 0x5000491C + 0, 0x0U);
		}
		// Try to power on network core
		nrf5340_release_network_core(target);
	}
}

bool nrf5340_probe(target_s *target)
{
	adiv5_access_port_s *ap = cortex_ap(target);
	if (ap == NULL)
		return false;

	if (ap->designer_code != 0x244 && ap->partno != 0x70)
		return false;

	if (ap->apsel >= 2)
		return false;

	adiv5_access_port_s *ctrl_ap = adiv5_new_ap(ap->dp, ap->apsel + 2);
	if (ctrl_ap == NULL || ctrl_ap->idr != NRF53_CTRL_AP_IDR)
		return false;

	data_s *data = malloc(sizeof(data_s));
	if (data == NULL) {
		adiv5_ap_unref(ctrl_ap);
		return false;
	}

	data->layout = &layouts[ap->apsel];
	data->ctrl_ap = ctrl_ap;
	target->target_storage = data;

	target->driver = data->layout->driver;
	target->target_options |= TOPT_INHIBIT_NRST;
	target->mass_erase = nrf5340_mass_erase;
	target->extended_reset = nrf5340_extended_reset;

	nrf5340_extended_reset(target);

	const uint32_t info_ram = ficr_read(target, FICR_INFO_RAM);

	target_add_ram32(target, data->layout->ram_base, info_ram * 1024U);

	const uint32_t codepage_size = ficr_read(target, FICR_INFO_CODEPAGE_SIZE);
	if (codepage_size != data->layout->codepage_size) {
		DEB("Unexpected codepage size for nrf5340: %" PRIx32 "\n", codepage_size);
		data_free(data);
		return false;
	}

	const uint32_t code_size = ficr_read(target, FICR_INFO_CODE_SIZE);
	if (code_size != data->layout->code_size) {
		DEB("Unexpected code size for nrf5340: %" PRIx32 "\n", code_size);
		data_free(data);
		return false;
	}

	nrf5340_add_flash(target, data->layout->flash_base, codepage_size * code_size, codepage_size);

	return true;
}

static bool ap_is_protected(adiv5_access_port_s *ap)
{
	return (adiv5_ap_read(ap, ADIV5_AP_CSW) & ADIV5_AP_CSW_DBGSWENABLE) == 0;
}

static bool dp_is_protected(adiv5_debug_port_s *dp, ap_type_e apsel)
{
	adiv5_access_port_s ap = {0};
	ap.dp = dp;
	ap.apsel = apsel;
	return ap_is_protected(&ap);
}

static bool ctrl_ap_erase_all(adiv5_access_port_s *ap)
{
	// Check if erase protect is enabled
	uint32_t erase_protect = adiv5_ap_read(ap, CTRL_ERASE_PROTECT_STATUS);
	if (erase_protect != 0) {
		// For now just error out. We should be able to turn that off, but we could brick
		// the thing so don't mess with the erase protect disable register.
		gdb_outf("unable to run erase all operation, erase protect is enabled\n");
		return false;
	}

	uint32_t status = adiv5_ap_read(ap, CTRL_ERASE_ALL_STATUS);
	if (status != 0) {
		gdb_outf("erase all operation aleardy in progress ?\n");
		return false;
	}

	adiv5_ap_write(ap, CTRL_ERASE_ALL, 0U);
	adiv5_ap_write(ap, CTRL_ERASE_ALL, 1U);
	do {
		status = adiv5_ap_read(ap, CTRL_ERASE_ALL_STATUS);
	} while (status == 1);

	adiv5_ap_write(ap, 0x0U, 1U);
	adiv5_ap_write(ap, 0x0U, 0U);

	adiv5_ap_write(ap, CTRL_ERASE_ALL, 0U);
	platform_delay(100);

	return true;
}

static bool dp_erase_all(adiv5_debug_port_s *dp, ap_type_e apsel)
{
	adiv5_access_port_s ap = {0};
	ap.dp = dp;
	ap.apsel = apsel;
	return ctrl_ap_erase_all(&ap);
}

static void recover_if_protected(adiv5_debug_port_s *dp)
{
	if (dp_is_protected(dp, ap_ahb_app)) {
		gdb_out("nRF53 App AP is protected, ");
		if (recover) {
			gdb_outf("recover is enabled, executing erase all operation\n");
			dp_erase_all(dp, ap_ctrl_app);
		} else {
			gdb_outf("run the recover command to erase all configuration and disable APPROTECT.\n");
		}
	}
	if (dp_is_protected(dp, ap_ahb_net)) {
		gdb_out("nRF53 App AP is protected, ");
		if (recover) {
			gdb_outf("recover is enabled, executing erase all operation\n");
			dp_erase_all(dp, ap_ctrl_net);
		} else {
			gdb_outf("run the recover command to erase all configuration and disable APPROTECT.\n");
		}
	}
}

void nrf5340_prepare(adiv5_debug_port_s *dp)
{
	recover_if_protected(dp);
}

static bool cmd_recover(target_s *, int, const char **)
{
	recover = true;
	return true;
}

const command_s platform_cmd_list[] = {{"recover", cmd_recover,
										   "Erases all user available non-volatile memory and disables the read back "
										   "protection mechanism if "
										   "enabled."},
	{NULL, NULL, NULL}};
