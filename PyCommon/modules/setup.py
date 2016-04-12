# +-------------------------------------------------------------------------
# | setup.py
# |
# | Author: Yoonsang Lee
# +-------------------------------------------------------------------------
# | COPYRIGHT:
# |    Copyright Yoonsang Lee 2013
# |    See the included COPYRIGHT.txt file for further details.
# |    
# |    This file is part of the DataDrivenBipedController.
# |    DataDrivenBipedController is free software: you can redistribute it and/or modify
# |    it under the terms of the MIT License.
# |
# |    You should have received a copy of the MIT License
# |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
# +-------------------------------------------------------------------------

from distutils.core import setup
from distutils.extension import Extension
from distutils.dir_util import copy_tree

boost_inc_dir = 'C:/Program Files (x86)/boost/boost_1_35_0'
boost_lib = 'boost_python-vc71-mt-1_35'
boost_lib_dir = 'C:/Program Files (x86)/boost/boost_1_35_0/lib'
bp_util_h = '../common_sources/bputil.h'

class ModuleInfo:
    def __init__(self):
        self.pkg_name = ''
        self.module_name = ''
        self.include_dirs = []
        self.libraries = []
        self.library_dirs = []
        self.sources_in_pkg_dir = []
        self.depends_in_pkg_dir = []
        self.additional_depends = []
        self.additional_sources = []

modules = []

m = ModuleInfo()

m = ModuleInfo()
modules.append(m)
m.pkg_name = 'Renderer'
m.module_name = 'csVpRenderer'
m.include_dirs = ['../external_libraries/VirtualPhysics/vpLib/include/']
m.libraries = ['opengl32', 'vpLib']
m.library_dirs = ['C:\Program Files\Microsoft Visual Studio .NET 2003\Vc7\PlatformSDK\Lib', 'Release']

m = ModuleInfo()
modules.append(m)
m.pkg_name = 'Simulator'
m.module_name = 'csVpModel'
m.include_dirs = ['../external_libraries/VirtualPhysics/vpLib/include/']
m.libraries = ['vpLib']
m.library_dirs = ['Release']
m.sources_in_pkg_dir = ['myGeom.cpp']
m.depends_in_pkg_dir = ['stdafx.h', 'myGeom.h']
m.additional_depends = ['../common_sources/vputil.h']

m = ModuleInfo()
modules.append(m)
m.pkg_name = 'Simulator'
m.module_name = 'csVpWorld'
m.include_dirs = ['../external_libraries/VirtualPhysics/vpLib/include/']
m.libraries = ['vpLib']
m.library_dirs = ['Release']
m.depends_in_pkg_dir = ['stdafx.h']
m.additional_depends = ['../common_sources/vputil.h']

m = ModuleInfo()
modules.append(m)
m.pkg_name = 'Simulator'
m.module_name = 'csVpBody'
m.include_dirs = ['../external_libraries/VirtualPhysics/vpLib/include/']
m.libraries = ['vpLib']
m.library_dirs = ['Release']
m.sources_in_pkg_dir = ['myGeom.cpp']
m.depends_in_pkg_dir = ['stdafx.h', 'myGeom.h']
m.additional_depends = ['../common_sources/vputil.h']

m = ModuleInfo()
modules.append(m)
m.pkg_name = 'Math'
m.module_name = 'csMath'
m.include_dirs = ['../external_libraries/VirtualPhysics/vpLib/include/']
m.sources_in_pkg_dir = ['EulerAngles.cpp']
m.depends_in_pkg_dir = ['stdafx.h', 'EulerAngles.h', 'QuatTypes.h']
m.additional_depends = ['../common_sources/vputil.h']


extensions = []
for m in modules:
    extensions.append(Extension(m.pkg_name + '.' + m.module_name,
                                sources = [m.pkg_name + '/' + m.module_name + '.cpp']\
                                        + [m.pkg_name + '/' + name for name in m.sources_in_pkg_dir]\
                                        + m.additional_sources,
                                include_dirs = [boost_inc_dir] + m.include_dirs,
                                libraries = [boost_lib] + m.libraries,
                                library_dirs = [boost_lib_dir] + m.library_dirs,
                                depends = [bp_util_h]
                                        + [m.pkg_name + '/' + m.module_name + '.h'] \
                                        + [m.pkg_name + '/' + name for name in m.depends_in_pkg_dir] \
                                        + m.additional_depends))

    
setup(name = 'Common', ext_modules = extensions)


copy_tree('build/lib.win32-2.5', './')






'''
csIMSModel = Extension('Implicit.csIMSModel',
                    sources = ['Implicit/csIMSModel.cpp'],
                    include_dirs = ['C:/Program Files/boost/boost_1_35_0'],
                    libraries = ['boost_python-vc71-mt-1_35', 'implicitMassSpringSolver2'],
                    library_dirs = ['C:/Program Files/boost/boost_1_35_0/lib', 'Release']
                    )

csIMSRenderer = Extension('Renderer.csIMSRenderer',
                    sources = ['Renderer/csIMSRenderer.cpp'],
                    include_dirs = ['C:/Program Files/boost/boost_1_35_0'],
                    libraries = ['boost_python-vc71-mt-1_35', 'opengl32'],
                    library_dirs = ['C:/Program Files/boost/boost_1_35_0/lib', 'C:\Program Files\Microsoft Visual Studio .NET 2003\Vc7\PlatformSDK\Lib']
                    )
setup(name = 'Common', ext_modules = [csIMSModel, csIMSRenderer])
'''
