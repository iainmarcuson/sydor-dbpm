# motorOmsAsyn
EPICS asyn motor drivers for the following [OMS](https://www.omsinmotion.com/) controllers: MAXv and MAXnet

motorOmsAsyn is a submodule of [motor](https://github.com/epics-modules/motor).  When motorOmsAsyn is built in the ``motor/modules`` directory, no manual configuration is needed.

motorOmsAsyn can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorOmsAsyn contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
