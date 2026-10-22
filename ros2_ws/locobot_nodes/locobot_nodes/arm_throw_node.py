        lbl.color.a = 1.0
        lbl.text = f'ball ({bx:.3f}, {by:.3f}, {bz:.3f})'
        markers.markers.append(lbl)

        self._marker_pub.publish(markers)

    # ── Joint Control ─────────────────────────────────────────────────────────

    def _p(self, pub, val):
        msg = Double()
        msg.data = float(val)
        pub.publish(msg)

    def _set_arm(self, w, q1, q2, q3, wr):
        self._p(self._pub_waist, w)
        self._p(self._pub_q1,   q1)
        self._p(self._pub_q2,   q2)
        self._p(self._pub_q3,   q3)
        self._p(self._pub_wr,   wr)
        self._cur_q1 = q1
        self._cur_q2 = q2
        self._cur_q3 = q3
        self._cur_w  = w

    def _set_arm_slow(self, w, q1, q2, q3, wr, steps=SLOW_STEPS, dur=SLOW_DURATION):
        self.get_logger().info(
            f'Slow({dur:.1f}s): q1:{self._cur_q1:.2f}→{q1:.2f} '
            f'q2:{self._cur_q2:.2f}→{q2:.2f} q3:{self._cur_q3:.2f}→{q3:.2f}'
        )
        dt = dur / steps
        for i in range(1, steps+1):
            t = i / steps
            self._p(self._pub_waist, self._cur_w  + t*(w  - self._cur_w))
            self._p(self._pub_q1,   self._cur_q1 + t*(q1 - self._cur_q1))
            self._p(self._pub_q2,   self._cur_q2 + t*(q2 - self._cur_q2))
            self._p(self._pub_q3,   self._cur_q3 + t*(q3 - self._cur_q3))
            self._p(self._pub_wr,   t * wr)
            time.sleep(dt)
        self._cur_q1 = q1
        self._cur_q2 = q2
        self._cur_q3 = q3
        self._cur_w  = w

    def _grip(self, pos):
        self._p(self._pub_lf,  pos)
        self._p(self._pub_rf, -pos)

    def _grip_slow(self, target, current, steps=15, dur=1.5):
        dt = dur / steps
        for i in range(1, steps+1):
            t = i / steps
            self._grip(current + t*(target - current))
            time.sleep(dt)

    # ── Main Sequence ─────────────────────────────────────────────────────────

    def _run_sequence(self):
        targets = self._pickup_targets()
        if targets is None:
            return
        pre, grasp = targets
        self._p(self._pub_roll, 0.0)

        # Step 1: Open gripper
        self.get_logger().info('PICKUP 1: Opening gripper...')
        self._grip_slow(GRIPPER_OPEN, GRIPPER_CLOSED, steps=10, dur=1.0)

        # Move above the estimated ball, then descend to its estimated center.
        self._set_arm_slow(*pre, WRIST_ROT_FIXED, steps=SLOW_STEPS, dur=SLOW_DURATION)
        self._set_arm_slow(*grasp, WRIST_ROT_FIXED, steps=SLOW_STEPS, dur=2.0)
        time.sleep(0.5)

        # Step 3: Close gripper tight
        self.get_logger().info('PICKUP 3: Closing gripper tight...')
        self._grip_slow(GRIPPER_TIGHT, GRIPPER_OPEN, steps=20, dur=2.0)
        time.sleep(0.5)
        self.get_logger().info('Gripper closure commanded; grasp is not sensor-verified.')

        # Lift along the approach, then return to the original throw posture.
        self._set_arm_slow(*pre, WRIST_ROT_FIXED, steps=SLOW_STEPS, dur=2.0)
        self._set_arm_slow(WAIST_FIXED, SHOULDER_HOME, ELBOW_HOME,
                           WRIST_HOME, WRIST_ROT_FIXED, steps=SLOW_STEPS, dur=2.5)

        # ── THROW ─────────────────────────────────────────────────────────────

        # Step 5: Wind-back
        self.get_logger().info(
            f'THROW 1: Wind-back | elbow={self._throw_elbow:.3f} rad'
        )
        self._set_arm(
            WAIST_FIXED, SHOULDER_WINDUP,
            self._throw_elbow, WRIST_HOME, WRIST_ROT_FIXED
        )
        time.sleep(self._wind_back_t)

        # Step 6: THROW!
        self.get_logger().info('THROW 2: THROWING! 🚀')
        self._set_arm(
            WAIST_FIXED, SHOULDER_THROW,
            self._throw_elbow, WRIST_HOME, WRIST_ROT_FIXED
        )
        time.sleep(0.15)

        # Step 7: Release
        self.get_logger().info('THROW 3: Releasing ball! 🎯')
        self._grip(GRIPPER_OPEN)
        time.sleep(self._throw_speed_t)

        # Step 8: Return home
        self.get_logger().info('THROW 4: Returning home.')
        self._set_arm(
            WAIST_FIXED, SHOULDER_HOME,
            ELBOW_HOME, WRIST_HOME, WRIST_ROT_FIXED
        )
        self._grip(GRIPPER_OPEN)

        self.get_logger().info('=== SEQUENCE COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = ArmThrowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
