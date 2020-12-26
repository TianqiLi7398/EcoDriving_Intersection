import unittest
from util import opt_vehicle, traffic_light

from state_time_opt import state_space

class TestStringMethods(unittest.TestCase):

    def test_time(self):
        
        m = 5   # m = light.loc - x_init / dx
        n = 23   # n = v_max - 1
        light_location = 50
        timeset = [15, 3, 20]                      # b, red, yellow, green
        timeset = [26, 5, 25]     
        dv, dt = 1, 0.5
        v_max = (n-1)*dv     
        delta_x = (light_location - 0.0) / m
        a_min, a_max = -5, 8
        t0_ = 28

        light = traffic_light(t0_, timeset, light_location)
        init_light = light.give_clock(0)
        init_vel = 15.0

        x0 = [0, init_vel, init_light, init_light]
        car = opt_vehicle(delta_x, v_max, a_max, a_min, light, x0, dt)

        obj = state_space(light, car)
        x, v, t, new_x, new_v = 50.0, 0.0, 20.0, 50.0, 0.0
        print(obj.light.give_clock(t), obj.validate_transition(x, v, t, new_x, new_v))

        for i in range(n):
            v = i * dv
            print(obj.potential_states(0, v))

    # def test_upper(self):
    #     self.assertEqual('foo'.upper(), 'FOO')

    # def test_isupper(self):
    #     self.assertTrue('FOO'.isupper())
    #     self.assertFalse('Foo'.isupper())

    # def test_split(self):
    #     s = 'hello world'
    #     self.assertEqual(s.split(), ['hello', 'world'])
    #     # check that s.split fails when the separator is not a string
    #     with self.assertRaises(TypeError):
    #         s.split(2)

if __name__ == '__main__':
    unittest.main()