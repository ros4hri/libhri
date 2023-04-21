^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri
^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* anonymous field as optional
* Contributors: lorenzoferrini

0.6.1 (2023-01-16)
------------------
* add callbacks for when speech is detected on a voice
* Contributors: Séverin Lemaignan

0.6.0 (2023-01-05)
------------------
* redefine hri::FeatureType enum to be used as bitmask
* Contributors: Séverin Lemaignan

0.5.3 (2022-10-26)
------------------
* bodies: expose the skeleton2d points
* package.xml: add libhri URL
* Contributors: Séverin Lemaignan, lorenzoferrini

0.5.2 (2022-10-10)
------------------
* expose the 3D transform of the voices
* expose face + gaze transform
* expose the 3D transform of the bodies
* minor refactor for safer access to engagement_status
* Contributors: Séverin Lemaignan

0.5.1 (2022-08-31)
------------------
* add comparision between 'feature trackers'
* update to new hri_msgs-0.8.0 names
* Contributors: Séverin Lemaignan

0.5.0 (2022-05-26)
------------------
* expose the current TF frame + transform of the person
* expose engagement status of people
* expose softbiometrics (age/gender) in faces
* add support for persons' aliases
  if a /humans/persons/<id>/alias points to another person id, libhri will use
  return the same pointer
* add callbacks when face/body/... are lost + support for known vs tracked persons
* add support for 'anonymous' persons
  Anonymous persons are persons that *may* disappear at any point.
  They are typically created because we *know* that a person is there (eg,
  we've detected a face), but that person is not yet permanently
  identified.
  API change: HriListener::getPersons() now returns *weak* pointers that
  need to be locked before being used.
* ensure the cropped and aligned face do not re-use the same underlying data
* add simple libhri example to display aligned faces
* remove spurious logging on cout
* Contributors: Séverin Lemaignan

0.4.3 (2022-04-28)
------------------
* fix gmock 'Call' syntax for older version of gmock (1.8). This was causing
  issues on ubuntu 18.04 (ROS melodic)
* Contributors: Séverin Lemaignan

0.4.2 (2022-04-27)
------------------
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
