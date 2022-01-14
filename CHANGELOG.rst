^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri
^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* replace hri_msgs::RegionOfInterestStamped by sensor_msgs::RegionOfInterest
  Follows changes in hri_msgs 0.2.0
* add skeleton of hri::Person class
* add empty Voice class
* expose the features' topic namespace + doc
* Contributors: Séverin Lemaignan

0.2.0 (2022-01-05)
------------------
* add basic support for bodies; only the RoIs for now
* Contributors: Séverin Lemaignan

0.1.0 (2022-01-05)
------------------
* use boost::optional for faces' features like RoI
* doc: setup rosdoc. Run `rosdoc_lite .` to generate
* test: expand the test suite
* cmake: explicit SYSTEM headers to avoid ROS shadowing issues
* Contributors: Séverin Lemaignan

0.0.3 (2022-01-05)
------------------
* do not try to compile hri_demo (internal test)
* Contributors: Séverin Lemaignan
