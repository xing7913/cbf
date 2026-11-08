import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1] / "src" / "safe_obstacle_avoidance"
sys.path.insert(0, str(ROOT))

from safe_obstacle_avoidance.reactive_planner import choose_command

class TestReactivePlanner(unittest.TestCase):
    def test_clear_path_moves_toward_goal(self):
        v,w,side,d = choose_command(np.full(100,4.0),0)
        self.assertGreater(v,0.1)
        self.assertAlmostEqual(w,0)
        self.assertEqual(side,0)

    def test_obstacle_prefers_open_left(self):
        image=np.full(100,3.0)
        image[38:62]=0.9
        image[62:86]=0.6
        v,w,side,d=choose_command(image,0)
        self.assertEqual(side,1)
        self.assertGreater(w,0)
        self.assertLess(v,0.15)

    def test_obstacle_prefers_open_right(self):
        image=np.full(100,3.0)
        image[38:62]=0.9
        image[14:38]=0.5
        _,w,side,_=choose_command(image,0)
        self.assertEqual(side,-1)
        self.assertLess(w,0)

    def test_close_obstacle_stops_linear_motion(self):
        a=np.full(100,0.3)
        v,w,_,_=choose_command(a,0)
        self.assertEqual(v,0)
        self.assertNotEqual(w,0)

    def test_invalid_depth_is_conservative(self):
        a=np.full(100,np.nan)
        v,_,_,_=choose_command(a,0)
        self.assertEqual(v,0)

    def test_goal_behind_turns_before_moving(self):
        v,w,_,_=choose_command(np.full(100,4.0),3.0)
        self.assertEqual(v,0)
        self.assertGreater(w,0)

    def test_short_input_stops(self):
        v,w,side,_=choose_command([1.0],0)
        self.assertEqual((v,w,side),(0,0,0))

if __name__=="__main__":
    unittest.main()
