^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri
^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add callback support for faces, bodies, voices, persons.
  Eg, call `onFace(cb)` to register callback invoked everytime a face is detected
* expose the aligned face in the Face class
* Contributors: Séverin Lemaignan

0.4.1 (2022-03-07)
------------------
* Fixed wrong feature subscribers indexing
* Contributors: lorenzoferrini

0.4.0 (2022-02-21)
------------------
* Facial Landmarks implementation
  Implementation of methods and structures required to access the
  facial landmarks
  Face Landmarks object size correction
* add tests for the person.face_id attribute
* actually subscribe to the person's face/body/voice id updates
* Contributors: Séverin Lemaignan, lorenzoferrini

0.3.1 (2022-02-07)
------------------
* add/update BSD license
* Contributors: Séverin Lemaignan

0.3.0 (2022-02-07)
------------------
* expose enum with the 4 feature types person,face,body,voice
* add voices and persons + improve const semantics
* Contributors: Séverin Lemaignan

0.2.3 (2022-01-21)
------------------
* Body::{getRoI->roi} + RoI not optional + add Body::cropped
* Contributors: Séverin Lemaignan

0.2.2 (2022-01-21)
------------------
* Face::{getRoI->roi} + RoI not optional + add Face::cropped
  In the latest revision of the ROS4HRI spec, the region of interest is
  always expected to be available (as well as the cropped face). As such,
  no point in using a boost::optional there.
* Contributors: Séverin Lemaignan

0.2.1 (2022-01-14)
------------------
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
