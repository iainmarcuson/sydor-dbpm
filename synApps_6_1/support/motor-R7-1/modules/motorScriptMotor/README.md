# motorScriptMotor
EPICS motor drivers for Lua ScriptMotor.  Requires the EPICS [lua](https://github.com/epics-modules/lua) module.

motorScriptMotor is a submodule of [motor](https://github.com/epics-modules/motor).  When motorScriptMotor is built in the ``motor/modules`` directory, no manual configuration is needed.

motorScriptMotor can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorScriptMotor contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
