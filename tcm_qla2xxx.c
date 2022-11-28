// SPDX-License-Identifier: GPL-2.0-or-later
/*******************************************************************************
 * This file contains tcm implementation using v4 configfs fabric infrastructure
 * for QLogic target mode HBAs
 *
 * (c) Copyright 2010-2013 Datera, Inc.
 *
 * Author: Nicholas A. Bellinger <nab@daterainc.com>
 *
 * tcm_qla2xxx_parse_wwn() and tcm_qla2xxx_format_wwn() contains code from
 * the TCM_FC / Open-FCoE.org fabric module.
 *
 * Copyright (c) 2010 Cisco Systems, Inc
 *
 ****************************************************************************/


#include <linux/module.h>

static int __init tcm_qla2xxx_init(void)
{
	return 0;
}

static void __exit tcm_qla2xxx_exit(void)
{
}

MODULE_DESCRIPTION("TCM QLA24XX+ series NPIV enabled fabric driver");
MODULE_LICENSE("GPL");
module_init(tcm_qla2xxx_init);
module_exit(tcm_qla2xxx_exit);
