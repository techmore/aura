/* SPDX-License-Identifier: MIT AND GPL-2.0-only */

#ifndef PCIE_H
#define PCIE_H

int pcie_prepare_fdt(void *dt);
int pcie_handoff_fdt(void *dt);
int pcie_init(void);
int pcie_shutdown(void);

/* Disable optional J713 PCIe admission after rail/controller/DART failure. */
int pcie_disable_j713_fdt(void *dt);
/* T8140: copy the APCIe PHY's tables from the ADT to the DT's PCIe node. */
int pcie_t8140_tables_fdt(void *dt);

#endif
