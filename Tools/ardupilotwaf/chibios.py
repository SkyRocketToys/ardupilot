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
    for name in ('cppflags', 'include_dirs'):
        print name
        tmp_str = bldnode.find_node(name).read()
        tmp_str = tmp_str.replace(';\n','')
        _dynamic_env_data[name] = re.split('; ', tmp_str)
        print _dynamic_env_data[name]

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)

    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['cppflags'])

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
        rule='BUILDDIR=${BUILDDIR}/modules/ChibiOS CHIBIOS=${CH_ROOT} AP_HAL=${HAL_CH_ROOT} ${MAKE} lib -f ${HAL_CH_ROOT}/hwdef/stm32f412_nucleo.mk',
        group='dynamic_sources',
        target='modules/ChibiOS/libch.a'
    )