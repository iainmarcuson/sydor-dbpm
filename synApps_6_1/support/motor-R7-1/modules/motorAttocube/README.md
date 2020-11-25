# motorAttocube
EPICS motor drivers for the following [Attocube](https://www.attocube.com) controllers: ANC150 Piezo Step Controller

motorAttocube is a submodule of [motor](https://github.com/epics-modules/motor).  When motorAttocube is built in the ``motor/modules`` directory, no manual configuration is needed.

motorAttocube can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorAttocube contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
