XDE-SwigISIRController
======================

Swig interface of the orcisir_ISIRController developed by the ISIR, over the
orc_framework developed by the CEA/LIST.

The module must be compiled with swig, no 'develop' mode is available.

Install:
--------

    python setup.py install [--user] [--prefix=PREFIX]

Common error:
-------------

 - When executing a simulation using XDE-SwigISIRController if the dynamic model is empty,
   the library xde-core-dev has not been correctly linked. 
   - Check the installation of xde-core-dev
   - Remove the `_xde_swig_isir_controller.so`
   - Try the setup script again

 - If `liborc_optim.so` cannot be found when a simulation is executed, check that the environment variable
   `$LD_LIBRARY_PATH` contains the directory where you have installed `orc_framework`
