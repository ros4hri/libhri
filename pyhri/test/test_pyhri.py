# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from cv_bridge import CvBridge
import cv2
from datetime import timedelta
from geometry_msgs.msg import TransformStamped
from hri_msgs.msg import (
    FacialActionUnits, FacialLandmarks, IdsList, LiveSpeech, NormalizedRegionOfInterest2D,
    Skeleton2D, SoftBiometrics)
from hri_msgs.msg import EngagementLevel as EngagementLevelMsg
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
import std_msgs.msg
from tf2_ros import StaticTransformBroadcaster
import unittest

from hri import (
    EngagementLevel, FacialActionUnit, FacialLandmark, Gender, HRIListener, SkeletalKeypoint)


class TestHRI(unittest.TestCase):
    latching_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE)

    @classmethod
    def setUpClass(cls) -> None:
        cls.context = Context()
        rclpy.init(context=cls.context)
        return super().setUpClass()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown(context=cls.context)
        return super().tearDownClass()

    def setUp(self) -> None:
        self.tester_node = rclpy.create_node('tester_node', context=self.context)
        self.hri_listener = HRIListener('hri_node', False)
        return super().setUp()

    def tearDown(self) -> None:
        try:
            del self.hri_listener
        except AttributeError:
            pass
        self.tester_node.destroy_node()
        return super().tearDown()

    def spin(self, hri_timeout_ms=100):
        self.hri_listener.spin_some(timedelta(milliseconds=hri_timeout_ms))

    def test_get_faces(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)

        self.assertEqual(faces_pub.get_subscription_count(), 1)
        self.assertEqual(len(self.hri_listener.faces), 0)

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        faces = self.hri_listener.faces
        self.assertEqual(len(faces), 1)
        self.assertIn('A', faces)
        self.assertEqual(faces['A'].id, 'A')

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(len(self.hri_listener.faces), 1)

        faces_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()
        faces = self.hri_listener.faces
        self.assertEqual(len(faces), 2)
        self.assertIn('A', faces)
        self.assertIn('B', faces)

        faces_pub.publish(IdsList(ids=['B']))
        self.spin()
        faces = self.hri_listener.faces
        self.assertEqual(len(faces), 1)
        self.assertNotIn('A', faces)
        self.assertIn('B', faces)

        faces_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(len(self.hri_listener.faces), 0)

        # we do not see the C++ reference count from Python, so we check if the desctuctor is
        # called for the shared object by checking the subscription count of /roi
        roi_b_pub = self.tester_node.create_publisher(
            NormalizedRegionOfInterest2D, '/humans/faces/B/roi', 1)
        del faces
        self.assertEqual(roi_b_pub.get_subscription_count(), 0)

        del self.hri_listener
        self.assertEqual(faces_pub.get_subscription_count(), 0)

    def test_get_faces_roi(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        roi_a_pub = self.tester_node.create_publisher(
            NormalizedRegionOfInterest2D, '/humans/faces/A/roi', 1)
        roi_b_pub = self.tester_node.create_publisher(
            NormalizedRegionOfInterest2D, '/humans/faces/B/roi', 1)

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(roi_a_pub.get_subscription_count(), 1)

        faces_pub.publish(IdsList(ids=['B']))
        self.spin()
        self.assertEqual(
            roi_a_pub.get_subscription_count(), 0,
            'Face A is deleted. No one should be subscribed to /humans/faces/A/roi anymore')
        self.assertEqual(
            roi_b_pub.get_subscription_count(), 1,
            'Face B should have subscribed to /humans/faces/B/roi')
        faces = self.hri_listener.faces
        self.assertIn('B', faces)
        face = faces['B']
        self.assertIsNotNone(face)
        self.assertEqual(face.ns, '/humans/faces/B')
        self.assertIsNone(face.roi)

        roi = NormalizedRegionOfInterest2D(xmin=0.1, ymin=0., xmax=1., ymax=1.)
        roi_b_pub.publish(roi)
        self.spin()
        self.assertIsNotNone(self.hri_listener.faces['B'].roi)
        self.assertAlmostEqual(self.hri_listener.faces['B'].roi[0], 0.1)

        roi.xmin = 0.2
        roi_b_pub.publish(roi)
        self.spin()
        self.assertAlmostEqual(self.hri_listener.faces['B'].roi[0], 0.2)

        faces_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()

        roi_a_pub.publish(roi)
        self.spin()
        faces = self.hri_listener.faces
        face_a = faces['A']
        face_b = faces['B']
        self.assertIsNotNone(face_a)
        self.assertIsNotNone(face_b)
        self.assertEqual(face_a.ns, '/humans/faces/A')
        self.assertIsNotNone(face_a.roi)
        self.assertAlmostEqual(face_a.roi[0], 0.2)
        self.assertEqual(face_b.ns, '/humans/faces/B')
        self.assertIsNotNone(face_b.roi)
        self.assertAlmostEqual(face_b.roi[0], 0.2)

    def test_get_bodies(self):
        bodies_pub = self.tester_node.create_publisher(IdsList, '/humans/bodies/tracked', 1)

        self.assertEqual(bodies_pub.get_subscription_count(), 1)
        self.assertEqual(len(self.hri_listener.bodies), 0)

        bodies_pub.publish(IdsList(ids=['A']))
        self.spin()
        bodies = self.hri_listener.bodies
        self.assertEqual(len(bodies), 1)
        self.assertIn('A', bodies)
        self.assertEqual(bodies['A'].id, 'A')

        bodies_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(len(self.hri_listener.bodies), 1)

        bodies_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()
        bodies = self.hri_listener.bodies
        self.assertEqual(len(bodies), 2)
        self.assertIn('A', bodies)
        self.assertIn('B', bodies)

        bodies_pub.publish(IdsList(ids=['B']))
        self.spin()
        bodies = self.hri_listener.bodies
        self.assertEqual(len(bodies), 1)
        self.assertNotIn('A', bodies)
        self.assertIn('B', bodies)

        bodies_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(len(self.hri_listener.bodies), 0)

        # we do not see the C++ reference count from Python, so we check if the destructor is
        # called for the shared object by checking the subscription count of /roi
        roi_b_pub = self.tester_node.create_publisher(
            NormalizedRegionOfInterest2D, '/humans/bodies/B/roi', 1)
        del bodies
        self.assertEqual(roi_b_pub.get_subscription_count(), 0)

        del self.hri_listener
        self.assertEqual(bodies_pub.get_subscription_count(), 0)

    def test_get_voices(self):
        voices_pub = self.tester_node.create_publisher(IdsList, '/humans/voices/tracked', 1)

        self.assertEqual(voices_pub.get_subscription_count(), 1)
        self.assertEqual(len(self.hri_listener.voices), 0)

        voices_pub.publish(IdsList(ids=['A']))
        self.spin()
        voices = self.hri_listener.voices
        self.assertEqual(len(voices), 1)
        self.assertIn('A', voices)
        self.assertEqual(voices['A'].id, 'A')

        voices_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(len(self.hri_listener.voices), 1)

        voices_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()
        voices = self.hri_listener.voices
        self.assertEqual(len(voices), 2)
        self.assertIn('A', voices)
        self.assertIn('B', voices)

        voices_pub.publish(IdsList(ids=['B']))
        self.spin()
        voices = self.hri_listener.voices
        self.assertEqual(len(voices), 1)
        self.assertNotIn('A', voices)
        self.assertIn('B', voices)

        voices_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(len(self.hri_listener.voices), 0)

        # we do not see the C++ reference count from Python, so we check if the desctuctor is
        # called for the shared object by checking the subscription count of /is_speaking
        is_speaking_b_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/voices/B/is_speaking', 1)
        del voices
        self.assertEqual(is_speaking_b_pub.get_subscription_count(), 0)

        del self.hri_listener
        self.assertEqual(voices_pub.get_subscription_count(), 0)

    def test_voice_callbacks(self):
        voices_pub = self.tester_node.create_publisher(IdsList, '/humans/voices/tracked', 1)
        voice_a_is_speaking_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/voices/A/is_speaking', 1)
        voice_a_speech_pub = self.tester_node.create_publisher(
            LiveSpeech, '/humans/voices/A/speech', 1)

        def cb(_):
            nonlocal cb_triggered
            cb_triggered = True

        def voice_cb(voice):
            nonlocal cb_triggered
            nonlocal cb
            cb_triggered = True
            voice.on_speaking(cb)
            voice.on_incremental_speech(cb)
            voice.on_speech(cb)

        self.hri_listener.on_voice(voice_cb)

        cb_triggered = False
        voices_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertTrue(cb_triggered)

        cb_triggered = False
        voice_a_is_speaking_pub.publish(std_msgs.msg.Bool(data=True))
        self.spin()
        self.assertTrue(cb_triggered)
        self.assertTrue(self.hri_listener.voices['A'].is_speaking)

        cb_triggered = False
        voice_a_is_speaking_pub.publish(std_msgs.msg.Bool(data=False))
        self.spin()
        self.assertTrue(cb_triggered)
        self.assertFalse(self.hri_listener.voices['A'].is_speaking)

        cb_triggered = False
        voice_a_speech_pub.publish(LiveSpeech(final='test speech'))
        self.spin()
        self.assertTrue(cb_triggered)
        self.assertEqual(self.hri_listener.voices['A'].speech, 'test speech')

        cb_triggered = False
        voice_a_speech_pub.publish(LiveSpeech(incremental='test speech incremental'))
        self.spin()
        self.assertTrue(cb_triggered)
        self.assertEqual(
            self.hri_listener.voices['A'].incremental_speech, 'test speech incremental')

    def test_get_known_persons(self):
        persons_pub = self.tester_node.create_publisher(IdsList, '/humans/persons/known', 1)

        self.assertEqual(persons_pub.get_subscription_count(), 1)
        self.assertEqual(len(self.hri_listener.persons), 0)

        persons_pub.publish(IdsList(ids=['A']))
        self.spin()
        persons = self.hri_listener.persons
        self.assertEqual(len(persons), 1)
        self.assertIn('A', persons)
        self.assertEqual(persons['A'].id, 'A')

        persons_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(len(self.hri_listener.persons), 1)

        persons_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()
        persons = self.hri_listener.persons
        self.assertEqual(len(persons), 2)
        self.assertIn('A', persons)
        self.assertIn('B', persons)

        persons_pub.publish(IdsList(ids=['B']))
        self.spin()
        persons = self.hri_listener.persons
        self.assertEqual(len(persons), 1)
        self.assertNotIn('A', persons)
        self.assertIn('B', persons)

        persons_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(len(self.hri_listener.persons), 0)

        # we do not see the C++ reference count from Python, so we check if the desctuctor is
        # called for the shared object by checking the subscription count of /anonymous
        anonymous_b_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/persons/B/anonymous', 1)
        del persons
        self.assertEqual(anonymous_b_pub.get_subscription_count(), 0)

        del self.hri_listener
        self.assertEqual(persons_pub.get_subscription_count(), 0)

    def test_get_tracked_persons(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)

        self.assertEqual(tracked_persons_pub.get_subscription_count(), 1)
        self.assertEqual(len(self.hri_listener.tracked_persons), 0)

        tracked_persons_pub.publish(IdsList(ids=['A']))
        self.spin()
        tracked_persons = self.hri_listener.tracked_persons
        self.assertEqual(len(tracked_persons), 1)
        self.assertIn('A', tracked_persons)
        self.assertEqual(tracked_persons['A'].id, 'A')

        tracked_persons_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(len(self.hri_listener.tracked_persons), 1)

        tracked_persons_pub.publish(IdsList(ids=['A', 'B']))
        self.spin()
        tracked_persons = self.hri_listener.tracked_persons
        self.assertEqual(len(tracked_persons), 2)
        self.assertIn('A', tracked_persons)
        self.assertIn('B', tracked_persons)

        tracked_persons_pub.publish(IdsList(ids=['B']))
        self.spin()
        tracked_persons = self.hri_listener.tracked_persons
        self.assertEqual(len(tracked_persons), 1)
        self.assertNotIn('A', tracked_persons)
        self.assertIn('B', tracked_persons)

        tracked_persons_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(len(self.hri_listener.tracked_persons), 0)

        # we do not see the C++ reference count from Python, so we check if the desctuctor is
        # called for the shared object by checking the subscription count of /anonymous
        anonymous_b_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/persons/B/anonymous', 1)
        del tracked_persons
        self.assertEqual(anonymous_b_pub.get_subscription_count(), 0)

        del self.hri_listener
        self.assertEqual(tracked_persons_pub.get_subscription_count(), 0)

    def test_person_attributes(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        p1_face_pub = self.tester_node.create_publisher(
            std_msgs.msg.String, '/humans/persons/p1/face_id', self.latching_qos)

        tracked_persons_pub.publish(IdsList(ids=['p1']))
        faces_pub.publish(IdsList(ids=['f1', 'f2']))
        self.spin()
        p1 = self.hri_listener.tracked_persons['p1']
        self.assertFalse(p1.anonymous, 'By default, persons are not supposed to be anonymous')
        self.assertIsNone(p1.face)

        p1_face_pub.publish(std_msgs.msg.String(data='f1'))
        self.spin()
        f1 = p1.face
        self.assertIsNotNone(f1)
        self.assertEqual(f1.id, 'f1')

    def test_anonymous_persons_and_aliases(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)
        faces_pub = self.tester_node.create_publisher(
            IdsList, '/humans/faces/tracked', 1)
        p1_anon_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/persons/p1/anonymous', self.latching_qos)
        p2_anon_pub = self.tester_node.create_publisher(
            std_msgs.msg.Bool, '/humans/persons/p2/anonymous', self.latching_qos)
        p1_face_pub = self.tester_node.create_publisher(
            std_msgs.msg.String, '/humans/persons/p1/face_id', self.latching_qos)
        p2_face_pub = self.tester_node.create_publisher(
            std_msgs.msg.String, '/humans/persons/p2/face_id', self.latching_qos)
        p2_alias_pub = self.tester_node.create_publisher(
            std_msgs.msg.String, '/humans/persons/p2/alias', self.latching_qos)

        tracked_persons_pub.publish(IdsList(ids=['p1', 'p2']))
        faces_pub.publish(IdsList(ids=['f1', 'f2']))
        self.spin()
        self.assertEqual(len(self.hri_listener.tracked_persons), 2)
        self.assertEqual(len(self.hri_listener.faces), 2)

        # each person is associated to a face
        p1_face_pub.publish(std_msgs.msg.String(data='f1'))
        p2_face_pub.publish(std_msgs.msg.String(data='f2'))
        p1_anon_pub.publish(std_msgs.msg.Bool(data=False))
        p2_anon_pub.publish(std_msgs.msg.Bool(data=True))
        self.spin()
        p1 = self.hri_listener.tracked_persons['p1']
        p2 = self.hri_listener.tracked_persons['p2']
        self.assertIsNotNone(p1.anonymous)
        self.assertIsNotNone(p2.anonymous)
        self.assertFalse(p1.anonymous)
        self.assertTrue(p2.anonymous)
        # being anonymous or not should have no impact on face associations
        self.assertEqual(p1.face.id, 'f1')
        self.assertEqual(p2.face.id, 'f2')

        # ALIASES

        # set p2 as an alias of p1
        p2_alias_pub.publish(std_msgs.msg.String(data='p1'))
        self.spin()
        self.assertEqual(len(self.hri_listener.tracked_persons), 2)
        p2 = self.hri_listener.tracked_persons['p2']
        self.assertEqual(p1, p2, 'p2 should now point to the same person as p1')
        self.assertEqual(p2.face.id, 'f1', "p2's face now points to f1")

        # remove the alias
        p2_alias_pub.publish(std_msgs.msg.String(data=''))
        self.spin()
        p2 = self.hri_listener.tracked_persons['p2']
        self.assertNotEqual(p1, p2, 'p2 is not anymore the same person as p1')
        self.assertEqual(p2.face.id, 'f2', "p2's face should still points to its former f2 face")

        # republish the alias
        p2_alias_pub.publish(std_msgs.msg.String(data='p1'))
        self.spin()
        p2 = self.hri_listener.tracked_persons['p2']
        self.assertEqual(p1, p2, 'p2 is again the same person as p1')

        # delete p1 -> p2 should be deleted as well
        tracked_persons_pub.publish(IdsList(ids=['p2']))
        self.spin()
        self.assertEqual(
            len(self.hri_listener.tracked_persons), 0,
            'The aliased person should have been deleted with its alias')

    def test_soft_biometrics(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)
        faces_pub = self.tester_node.create_publisher(
            IdsList, '/humans/faces/tracked', 1)
        p1_face_pub = self.tester_node.create_publisher(
            std_msgs.msg.String, '/humans/persons/p1/face_id', self.latching_qos)
        softbiometrics_pub = self.tester_node.create_publisher(
            SoftBiometrics, '/humans/faces/f1/softbiometrics', 1)

        tracked_persons_pub.publish(IdsList(ids=['p1']))
        faces_pub.publish(IdsList(ids=['f1']))
        self.spin()
        self.assertEqual(len(self.hri_listener.tracked_persons), 1)
        self.assertEqual(len(self.hri_listener.faces), 1)

        softbiometrics_msg = SoftBiometrics(
            age=45, age_confidence=0.8, gender=SoftBiometrics.FEMALE, gender_confidence=0.7)
        softbiometrics_pub.publish(softbiometrics_msg)
        p1_face_pub.publish(std_msgs.msg.String(data='f1'))
        self.spin()
        face = self.hri_listener.tracked_persons['p1'].face
        self.assertIsNotNone(face.age)
        self.assertEqual(face.age, 45)
        self.assertIsNotNone(face.gender)
        self.assertEqual(face.gender, Gender.FEMALE)

        softbiometrics_msg.gender = SoftBiometrics.OTHER
        softbiometrics_pub.publish(softbiometrics_msg)
        self.spin()
        self.assertEqual(face.gender, Gender.OTHER)

        softbiometrics_msg.gender = SoftBiometrics.UNDEFINED
        softbiometrics_pub.publish(softbiometrics_msg)
        self.spin()
        self.assertIsNone(face.gender)

    def test_engagement_level(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)
        engagement_pub = self.tester_node.create_publisher(
            EngagementLevelMsg, '/humans/persons/p1/engagement_status', 1)

        tracked_persons_pub.publish(IdsList(ids=['p1']))
        self.spin()
        p1 = self.hri_listener.tracked_persons['p1']
        engagement_pub.publish(EngagementLevelMsg(level=EngagementLevelMsg.DISENGAGED))
        self.spin()
        self.assertIsNotNone(p1.engagement_status)
        self.assertEqual(p1.engagement_status, EngagementLevel.DISENGAGED)

        engagement_pub.publish(EngagementLevelMsg(level=EngagementLevelMsg.ENGAGED))
        self.spin()
        self.assertEqual(p1.engagement_status, EngagementLevel.ENGAGED)

        engagement_pub.publish(EngagementLevelMsg(level=EngagementLevelMsg.UNKNOWN))
        self.spin()
        self.assertIsNone(p1.engagement_status)

    def test_image(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        face_cropped_a_pub = self.tester_node.create_publisher(
            Image, '/humans/faces/A/cropped', 1)
        image = np.random.randint(0, 255, size=(64, 64, 3), dtype=np.uint8)
        image_msg = CvBridge().cv2_to_imgmsg(image, 'bgr8', std_msgs.msg.Header())

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(face_cropped_a_pub.get_subscription_count(), 1)
        face_a = self.hri_listener.faces['A']
        self.assertIsNone(face_a.cropped)

        face_cropped_a_pub.publish(image_msg)
        self.spin()
        self.assertIsNotNone(face_a.cropped)
        self.assertAlmostEqual(cv2.norm(face_a.cropped, image), 0.)

    def test_facial_action_units(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        fau_a_pub = self.tester_node.create_publisher(
            FacialActionUnits, '/humans/faces/A/facs', 1)
        fau_msg = FacialActionUnits()

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(fau_a_pub.get_subscription_count(), 1)
        face_a = self.hri_listener.faces['A']
        self.assertIsNone(face_a.facial_action_units)

        fau_msg.intensity[fau_msg.WINK] = 0.5
        fau_msg.confidence[fau_msg.WINK] = 0.8
        fau_a_pub.publish(fau_msg)
        self.spin()
        self.assertIsNotNone(face_a.facial_action_units)
        fau = face_a.facial_action_units[FacialActionUnit.WINK]
        self.assertAlmostEqual(fau[0], 0.5)
        self.assertAlmostEqual(fau[1], 0.8)

        fau_msg.intensity[fau_msg.WINK] = 0.0
        fau_msg.confidence[fau_msg.WINK] = 1.0
        fau_a_pub.publish(fau_msg)
        self.spin()
        fau = face_a.facial_action_units[FacialActionUnit.WINK]
        self.assertAlmostEqual(fau[0], 0.0)
        self.assertAlmostEqual(fau[1], 1.0)

    def test_facial_landmarks(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        facial_landmarks_a_pub = self.tester_node.create_publisher(
            FacialLandmarks, '/humans/faces/A/landmarks', 1)
        facial_landmarks_msg = FacialLandmarks()

        faces_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(facial_landmarks_a_pub.get_subscription_count(), 1)
        face_a = self.hri_listener.faces['A']
        self.assertIsNone(face_a.facial_landmarks)

        facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x = 0.3
        facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y = 0.5
        facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c = 0.8
        facial_landmarks_a_pub.publish(facial_landmarks_msg)
        self.spin()
        self.assertIsNotNone(face_a.facial_landmarks)
        point = face_a.facial_landmarks[FacialLandmark.NOSE]
        self.assertAlmostEqual(
            point[0], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x)
        self.assertAlmostEqual(
            point[1], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y)
        self.assertAlmostEqual(
            point[2], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c)

        facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x = 1.0
        facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c = 0.0
        facial_landmarks_a_pub.publish(facial_landmarks_msg)
        self.spin()
        point = face_a.facial_landmarks[FacialLandmark.NOSE]
        self.assertAlmostEqual(
            point[0], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x)
        self.assertAlmostEqual(
            point[1], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y)
        self.assertAlmostEqual(
            point[2], facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c)

    def test_skeletal_keypoints(self):
        bodies_pub = self.tester_node.create_publisher(IdsList, '/humans/bodies/tracked', 1)
        body_keypoints_a_pub = self.tester_node.create_publisher(
            Skeleton2D, '/humans/bodies/A/skeleton2d', 1)
        skeleton_msg = Skeleton2D()

        bodies_pub.publish(IdsList(ids=['A']))
        self.spin()
        self.assertEqual(body_keypoints_a_pub.get_subscription_count(), 1)
        body_a = self.hri_listener.bodies['A']
        self.assertIsNone(body_a.skeleton)

        skeleton_msg.skeleton[skeleton_msg.NOSE].x = 0.3
        skeleton_msg.skeleton[skeleton_msg.NOSE].y = 0.5
        skeleton_msg.skeleton[skeleton_msg.NOSE].c = 0.8
        body_keypoints_a_pub.publish(skeleton_msg)
        self.spin()
        self.assertIsNotNone(body_a.skeleton)
        point = body_a.skeleton[SkeletalKeypoint.NOSE]
        self.assertAlmostEqual(
            point[0], skeleton_msg.skeleton[skeleton_msg.NOSE].x)
        self.assertAlmostEqual(
            point[1], skeleton_msg.skeleton[skeleton_msg.NOSE].y)
        self.assertAlmostEqual(
            point[2], skeleton_msg.skeleton[skeleton_msg.NOSE].c)

        skeleton_msg.skeleton[skeleton_msg.NOSE].x = 1.0
        skeleton_msg.skeleton[skeleton_msg.NOSE].c = 0.0
        body_keypoints_a_pub.publish(skeleton_msg)
        self.spin()
        point = body_a.skeleton[SkeletalKeypoint.NOSE]
        self.assertAlmostEqual(
            point[0], skeleton_msg.skeleton[skeleton_msg.NOSE].x)
        self.assertAlmostEqual(
            point[1], skeleton_msg.skeleton[skeleton_msg.NOSE].y)
        self.assertAlmostEqual(
            point[2], skeleton_msg.skeleton[skeleton_msg.NOSE].c)

    def test_callbacks(self):
        faces_pub = self.tester_node.create_publisher(IdsList, '/humans/faces/tracked', 1)
        bodies_pub = self.tester_node.create_publisher(IdsList, '/humans/bodies/tracked', 1)
        voices_pub = self.tester_node.create_publisher(IdsList, '/humans/voices/tracked', 1)
        persons_pub = self.tester_node.create_publisher(IdsList, '/humans/persons/known', 1)
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)

        face_cb_invoked = 0
        face_lost_cb_invoked = 0
        body_cb_invoked = 0
        body_lost_cb_invoked = 0
        voice_cb_invoked = 0
        voice_lost_cb_invoked = 0
        person_cb_invoked = 0
        person_lost_cb_invoked = 0
        tracked_person_cb_invoked = 0
        tracked_person_lost_cb_invoked = 0

        def face_cb(_):
            nonlocal face_cb_invoked
            face_cb_invoked = face_cb_invoked + 1

        def face_lost_cb(_):
            nonlocal face_lost_cb_invoked
            face_lost_cb_invoked = face_lost_cb_invoked + 1

        def body_cb(_):
            nonlocal body_cb_invoked
            body_cb_invoked = body_cb_invoked + 1

        def body_lost_cb(_):
            nonlocal body_lost_cb_invoked
            body_lost_cb_invoked = body_lost_cb_invoked + 1

        def voice_cb(_):
            nonlocal voice_cb_invoked
            voice_cb_invoked = voice_cb_invoked + 1

        def voice_lost_cb(_):
            nonlocal voice_lost_cb_invoked
            voice_lost_cb_invoked = voice_lost_cb_invoked + 1

        def person_cb(_):
            nonlocal person_cb_invoked
            person_cb_invoked = person_cb_invoked + 1

        def person_lost_cb(_):
            nonlocal person_lost_cb_invoked
            person_lost_cb_invoked = person_lost_cb_invoked + 1

        def tracked_person_cb(_):
            nonlocal tracked_person_cb_invoked
            tracked_person_cb_invoked = tracked_person_cb_invoked + 1

        def tracked_person_lost_cb(_):
            nonlocal tracked_person_lost_cb_invoked
            tracked_person_lost_cb_invoked = tracked_person_lost_cb_invoked + 1

        self.hri_listener.on_face(face_cb)
        self.hri_listener.on_face_lost(face_lost_cb)
        self.hri_listener.on_body(body_cb)
        self.hri_listener.on_body_lost(body_lost_cb)
        self.hri_listener.on_voice(voice_cb)
        self.hri_listener.on_voice_lost(voice_lost_cb)
        self.hri_listener.on_person(person_cb)
        self.hri_listener.on_person_lost(person_lost_cb)
        self.hri_listener.on_tracked_person(tracked_person_cb)
        self.hri_listener.on_tracked_person_lost(tracked_person_lost_cb)

        old_face_cb_invoked = face_cb_invoked
        old_face_lost_cb_invoked = face_lost_cb_invoked
        faces_pub.publish(IdsList(ids=['id1']))
        self.spin()
        self.assertEqual(face_cb_invoked, old_face_cb_invoked + 1)
        self.assertEqual(face_lost_cb_invoked, old_face_lost_cb_invoked)

        old_face_cb_invoked = face_cb_invoked
        faces_pub.publish(IdsList(ids=['id1', 'id2']))
        self.spin()
        self.assertEqual(face_cb_invoked, old_face_cb_invoked + 1)

        old_face_cb_invoked = face_cb_invoked
        old_face_lost_cb_invoked = face_lost_cb_invoked
        faces_pub.publish(IdsList(ids=['id3', 'id4']))
        self.spin()
        self.assertEqual(face_cb_invoked, old_face_cb_invoked + 2)
        self.assertEqual(face_lost_cb_invoked, old_face_lost_cb_invoked + 2)

        old_body_cb_invoked = body_cb_invoked
        old_body_lost_cb_invoked = body_lost_cb_invoked
        bodies_pub.publish(IdsList(ids=['id1', 'id2']))
        self.spin()
        self.assertEqual(body_cb_invoked, old_body_cb_invoked + 2)
        self.assertEqual(body_lost_cb_invoked, old_body_lost_cb_invoked)

        old_face_cb_invoked = face_cb_invoked
        old_face_lost_cb_invoked = face_lost_cb_invoked
        old_body_cb_invoked = body_cb_invoked
        old_body_lost_cb_invoked = body_lost_cb_invoked
        ids = ['id1', 'id2', 'id3']
        faces_pub.publish(IdsList(ids=ids))
        bodies_pub.publish(IdsList(ids=ids))
        self.spin()
        self.assertEqual(face_cb_invoked, old_face_cb_invoked + 2)
        self.assertEqual(face_lost_cb_invoked, old_face_lost_cb_invoked + 1)
        self.assertEqual(body_cb_invoked, old_body_cb_invoked + 1)
        self.assertEqual(body_lost_cb_invoked, old_body_lost_cb_invoked)

        old_face_cb_invoked = face_cb_invoked
        old_face_lost_cb_invoked = face_lost_cb_invoked
        old_body_cb_invoked = body_cb_invoked
        old_body_lost_cb_invoked = body_lost_cb_invoked
        ids = ['id5', 'id6', 'id7']
        faces_pub.publish(IdsList(ids=ids))
        bodies_pub.publish(IdsList(ids=ids))
        self.spin()
        self.assertEqual(face_cb_invoked, old_face_cb_invoked + 3)
        self.assertEqual(face_lost_cb_invoked, old_face_lost_cb_invoked + 3)
        self.assertEqual(body_cb_invoked, old_body_cb_invoked + 3)
        self.assertEqual(body_lost_cb_invoked, old_body_lost_cb_invoked + 3)

        old_voice_cb_invoked = voice_cb_invoked
        old_voice_lost_cb_invoked = voice_lost_cb_invoked
        old_person_cb_invoked = person_cb_invoked
        old_person_lost_cb_invoked = person_lost_cb_invoked
        old_tracked_person_cb_invoked = tracked_person_cb_invoked
        old_tracked_person_lost_cb_invoked = tracked_person_lost_cb_invoked
        voices_pub.publish(IdsList(ids=['id1', 'id2', 'id3']))
        persons_pub.publish(IdsList(ids=['id1', 'id2']))
        tracked_persons_pub.publish(IdsList(ids=['id1']))
        self.spin()
        self.assertEqual(voice_cb_invoked, old_voice_cb_invoked + 3)
        self.assertEqual(voice_lost_cb_invoked, old_voice_lost_cb_invoked)
        self.assertEqual(person_cb_invoked, old_person_cb_invoked + 2)
        self.assertEqual(person_lost_cb_invoked, old_person_lost_cb_invoked)
        self.assertEqual(tracked_person_cb_invoked, old_tracked_person_cb_invoked + 1)
        self.assertEqual(tracked_person_lost_cb_invoked, old_tracked_person_lost_cb_invoked)

        old_voice_cb_invoked = voice_cb_invoked
        old_voice_lost_cb_invoked = voice_lost_cb_invoked
        old_person_cb_invoked = person_cb_invoked
        old_person_lost_cb_invoked = person_lost_cb_invoked
        old_tracked_person_cb_invoked = tracked_person_cb_invoked
        old_tracked_person_lost_cb_invoked = tracked_person_lost_cb_invoked
        voices_pub.publish(IdsList(ids=[]))
        persons_pub.publish(IdsList(ids=[]))
        tracked_persons_pub.publish(IdsList(ids=[]))
        self.spin()
        self.assertEqual(voice_cb_invoked, old_voice_cb_invoked)
        self.assertEqual(voice_lost_cb_invoked, old_voice_lost_cb_invoked + 3)
        self.assertEqual(person_cb_invoked, old_person_cb_invoked)
        self.assertEqual(person_lost_cb_invoked, old_person_lost_cb_invoked + 2)
        self.assertEqual(tracked_person_cb_invoked, old_tracked_person_cb_invoked)
        self.assertEqual(tracked_person_lost_cb_invoked, old_tracked_person_lost_cb_invoked + 1)

    def test_people_location(self):
        tracked_persons_pub = self.tester_node.create_publisher(
            IdsList, '/humans/persons/tracked', 1)
        location_confidence_pub = self.tester_node.create_publisher(
            std_msgs.msg.Float32, '/humans/persons/p1/location_confidence', 1)
        tester_executor = SingleThreadedExecutor(context=self.context)
        tester_executor.add_node(self.tester_node)
        static_broadcaster = StaticTransformBroadcaster(self.tester_node)
        transform_msg = TransformStamped()

        self.hri_listener.set_reference_frame('base_link')
        transform_msg.header.stamp = self.tester_node.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'world'
        transform_msg.child_frame_id = 'base_link'
        transform_msg.transform.translation.x = -1.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation.w = 1.0
        static_broadcaster.sendTransform(transform_msg)
        tester_executor.spin_once(1.)
        self.spin()

        tracked_persons_pub.publish(IdsList(ids=['p1']))
        self.spin()
        p1 = self.hri_listener.tracked_persons['p1']

        location_confidence_pub.publish(std_msgs.msg.Float32(data=0.))
        self.spin()
        self.assertIsNotNone(p1.location_confidence)
        self.assertAlmostEqual(p1.location_confidence, 0.)
        self.assertIsNone(
            p1.transform, 'location confidence at 0, no transform should be available')

        location_confidence_pub.publish(std_msgs.msg.Float32(data=0.5))
        self.spin()
        self.assertAlmostEqual(p1.location_confidence, 0.5)
        self.assertIsNone(
            p1.transform,
            'location confidence > 0 but no transform published yet -> \
                no transform should be returned')

        transform_msg.child_frame_id = 'person_p1'
        transform_msg.transform.translation.x = transform_msg.transform.translation.x + 2.0
        static_broadcaster.sendTransform(transform_msg)
        tester_executor.spin_once(1.)
        self.spin()
        self.assertIsNotNone(
            p1.transform, 'location confidence > 0 => a transform should be available')
        t = p1.transform
        self.assertEqual(t.child_frame_id, 'person_p1')
        self.assertEqual(t.header.frame_id, 'base_link')
        self.assertAlmostEqual(t.transform.translation.x, 2.0)

        self.hri_listener.set_reference_frame('person_p1')
        self.assertIsNotNone(p1.transform)
        t = p1.transform
        self.assertEqual(t.child_frame_id, 'person_p1')
        self.assertEqual(t.header.frame_id, 'person_p1')
        self.assertAlmostEqual(t.transform.translation.x, 0.)

        location_confidence_pub.publish(std_msgs.msg.Float32(data=1.0))
        self.spin()
        self.assertAlmostEqual(p1.location_confidence, 1.0)
        self.assertIsNotNone(
            p1.transform, 'location confidence > 0 => a transform should be available')


if __name__ == '__main__':
    unittest.main()
