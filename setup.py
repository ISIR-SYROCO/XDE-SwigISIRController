#! /usr/bin/env python

# System imports
from distutils.core import setup, Extension
from distutils.command.build_py import build_py
import commands
import os



flag_map = {'-I': 'include_dirs', '-L': 'library_dirs', '-l': 'libraries'}

def get_additional_include_dir_from_env():
    kw = {'include_dirs':[], 'library_dirs':[], 'libraries':[]}
    for token in commands.getoutput("echo $CPLUS_INCLUDE_PATH").split(":"):
        if token != "":
            kw['include_dirs'].append(token)
    return kw

# define pkgconfig return data
def pkgconfig(pkgname, required=True):

    if commands.getoutput("pkg-config --exists "+pkgname+" ; echo $?") == "0":
        print "-- "+pkgname + " found"
    else:
        print "-- "+pkgname + " not found"
        if required:
            raise ValueError("  -- This package ("+pkgname+") is required.")
        return None
    kw = {'include_dirs':[], 'library_dirs':[], 'libraries':[], "compiler_options":[]}
    for token in commands.getoutput("pkg-config --libs --cflags "+pkgname).split():
        if token[:2] in flag_map:
            kw[flag_map[token[:2]]].append(token[2:])
#        else:
#            print "token: '"+token+"' not taken into account"
    return kw


def get_package_from_build_type(pkgname, required=True, Debug=False, dbg_postfix="_dbg"):
    if Debug:
        res = pkgconfig(pkgname+dbg_postfix, False)
        if res is None:
            print "-- looking for release version of package:"
            return pkgconfig(pkgname+dbg_postfix, required)
        else:
            return res
    else:
        return pkgconfig(pkgname+dbg_postfix, required)


def get_packages_data(lpkg):
    kw = {'include_dirs':[], 'library_dirs':[], 'libraries':[]}
    for pkg in lpkg:
        for n in kw:
            kw[n].extend(pkg[n])
    return kw




# try to find interesting module
eigen_lgsm             = pkgconfig("eigen_lgsm", True)
quadprog               = pkgconfig("quadprog", True)
orc_framework          = pkgconfig("orc_framework", True)
orcisir_ISIRController = pkgconfig("orcisir_ISIRController", True)
xdecore                = pkgconfig("xdecore", False)
additional             = get_additional_include_dir_from_env()

orocos                 = pkgconfig("orocos-rtt-gnulinux", False)

# gather all data
other_swig_opt = []
other_compiler_args = []
packages_data = get_packages_data([eigen_lgsm, quadprog, orc_framework, orcisir_ISIRController, additional])


# check for optional package: xde
if xdecore is not None:
    xde_data = get_packages_data([xdecore])
    for elem in ["include_dirs", "library_dirs", "libraries"]:
        packages_data[elem].extend(xde_data[elem])
    other_swig_opt.append("-DXDECORE_IS_AVAILABLE")


# check for optional package: orocos
if orocos is not None:
    incdir = commands.getoutput("pkg-config --variable=includedir orocos-rtt-gnulinux")
    libdir = commands.getoutput("pkg-config --variable=libdir orocos-rtt-gnulinux")
    packages_data["include_dirs"].extend([incdir, incdir+os.sep+"rtt"])
    packages_data["library_dirs"].extend([libdir])
    packages_data["libraries"].extend(["orocos-rtt-gnulinux"])
    other_compiler_args.extend(["-DOROCOS_TARGET=gnulinux"])
    other_swig_opt.append("-DOROCOS_IS_AVAILABLE")



# SwigISIRController
_swig_swig_isir_controller = Extension("swig_isir_controller._swig_isir_controller",
                   ["src/swig_isir_controller.i"],
                   swig_opts = ["-c++"] + ["-I"+p for p in packages_data['include_dirs']] + other_swig_opt,
                   extra_compile_args = ["-fpermissive"] + other_compiler_args,
                   **packages_data #include, libs
                   )



#to force the package building extension before all we change the script_args list:
import sys
script_args= ["build_ext"] + sys.argv[1:] # To force a first build of the Extension(s)

# py setup
dist = setup(name   = "swig_isir_controller",
        description = "Swig library to all the element available in orcisir_ISIRController.",
        author      = "Joseph Salini",
        url         = 'https://github.com/XDE-ISIR/XDE-SwigISIRController',
        version     = "0.1",
        ext_modules = [_swig_swig_isir_controller],
        packages    = ["swig_isir_controller"],
        package_dir = {'swig_isir_controller':'src'},

        script_args = script_args,
        )


