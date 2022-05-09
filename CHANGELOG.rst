^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2022-05-09)
------------------
* add front-facing realsense to x500
* updated mbzirc sensor package
* refactored mavros launchfiles
* + install in cmakelists
* partial merge of marine branch
* Fix roll limits
* Update servo camera xacros
* update naki servo camera daes and their origins
* naki: split gimbal from frame to enable movable visual
* Update servo camera macro
  Add NAKI II drone to simulation
* ouster diag topic defaults now to os_cloud_nodelet/is_alive to match hw driver
* fix naki propellers rotation
* add safety led macro
* add macro for ultrasonic sensors; add them to NAKI drone
* add models/visuals/sensor setups for NAKI drone
* fixed whycon box for f450
* update motor constants
  random
* initial commit with naki files: copy of f450, modeless, octo_coax
* new realsense loading, fixed multiple instances
* Revert "updating realsense naming"
  This reverts commit 96156a3f6dbff2c2a64157790c8bb8c64ec1204c.
* updating realsense naming
* random
* rotated vio IMU to correspond with the real drone
* add timepix3 sensor to x500 and t650 (`#18 <https://github.com/ctu-mrs/mrs_simulation/issues/18>`_)
* added realsense front to f330
* update brus spawner options
* added missing tag to t650
* added enable-light option to x500
* added x500 ouster support
* add brus to spawner (`#17 <https://github.com/ctu-mrs/mrs_simulation/issues/17>`_)
* added ground truth plugin to all uavs, added custom mag plugin
* Contributors: Dan Hert, DanHert, Matej Petrlik, Pavel Petracek, Pavel Petráček, Tomas Baca, Vaclav Pritzl, Vit Kratky, stibipet

1.0.1 (2021-05-16)
------------------
* increased timeouts in drone spawner to 60s
* add rad120 to component_snippets.xacro
* Adding the third/back camera to simulation
* enabled ouster for eaglemk2
* Merge branch 'uvdar_better_signalling'
* added realsense_up_down parameter for 2 realsense cameras pointing up and down
* Fixing the camera frametate at 60Hz
* Adding signal_id parameter to component snippets
* Changing the frequency parameter to signal id parameter
* Contributors: Matouš Vrba, Pavel Petracek, Tomas Baca, Vaclav Pritzl, Viktor Walter, ViktorWalter

1.0.0 (2021-03-18)
------------------
* Major release
