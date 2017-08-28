#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    tmp_str = bldnode.find_node('include_dirs').read()
    tmp_str = tmp_str.replace(';\n','')
    if 'include_dirs' == 'include_dirs':
        tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    _dynamic_env_data['include_dirs'] = re.split('; ', tmp_str)
    print _dynamic_env_data['include_dirs']

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)
    self.use += ' ch'
    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])

def configure(cfg):
    cfg.find_program('make', var='MAKE')
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CH_ROOT = srcpath('modules/ChibiOS')
    env.HAL_CH_ROOT = srcpath('libraries/AP_HAL_ChibiOS')
    env.BUILDDIR = bldpath('.')

def build(bld):
    bld(
        rule='touch Makefile && BUILDDIR=${BUILDDIR}/modules/ChibiOS CHIBIOS=${CH_ROOT} AP_HAL=${HAL_CH_ROOT} ${MAKE} lib -f ${HAL_CH_ROOT}/hwdef/stm32f412_nucleo.mk',
        group='dynamic_sources',
        target='modules/ChibiOS/libch.a'
    )
    bld.env.STLIB += ['ch']
    bld.env.STLIBPATH += ['modules/ChibiOS/']
# class make_chibios(Task.Task):
#     run_str = 'make all -f ${AP_HAL}/hwdef/stm32f412_nucleo.mk'
#     color = 'CYAN'

#     def keyword(self):
#         return "ChibiOS: Running make to link APM to ChibiOS"
#     def __str__(self):
#         return self.outputs[0].path_from(self.generator.bld.bldnode)

# @feature('ch_ap_program')
# @after_method('process_source')
# def chibios_firmware(self):
#     fw_task = self.create_task('make_chibios', self.bld.bldnode.make_node(self.program_dir), self.bld.bldnode.make_node(self.program_name))
#     fw_task.set_run_after(self.link_task)
#     fw_task.env.env = dict(os.environ)
#     fw_task.env.env['BUILDDIR'] = self.bld.bldnode.find_or_declare('modules/ChibiOS').abspath()
#     fw_task.env.env['CH_ROOT'] = self.bld.srcnode.make_node('modules/ChibiOS').abspath()
#     fw_task.env['AP_HAL'] = self.bld.srcnode.make_node('libraries/AP_HAL_ChibiOS').abspath()
