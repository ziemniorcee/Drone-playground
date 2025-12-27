import pybullet as p


class InputManager:
    def __init__(self):
        # Configurable sensitivities
        self.yaw_rate = 0.02
        self.alt_rate = 0.05
        self.pitch_val = 0.5
        self.roll_val = 0.5

    def get_commands(self):
        """
        Returns a dictionary of commands based on current key presses.
        Structure: {'roll': float, 'pitch': float, 'yaw': float, 'alt': float, 'reset': bool}
        """
        keys = p.getKeyboardEvents()

        commands = {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'alt': 0,
            'reset': False,
            'camera_toggle': False,
        }

        # --- Reset ---
        if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
            commands['reset'] = True

        if ord('z') in keys and (keys[ord('z')] & p.KEY_WAS_TRIGGERED):
            commands['camera_toggle'] = True

        # --- Altitude (Shift/Ctrl) ---
        if p.B3G_SHIFT in keys and (keys[p.B3G_SHIFT] & p.KEY_IS_DOWN):
            commands['alt'] = self.alt_rate
        if p.B3G_CONTROL in keys and (keys[p.B3G_CONTROL] & p.KEY_IS_DOWN):
            commands['alt'] = -self.alt_rate

        # --- Yaw (A/D) ---
        if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN):
            commands['yaw'] = self.yaw_rate
        if ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN):
            commands['yaw'] = -self.yaw_rate

        # --- Pitch (Arrow Up/Down) ---
        if p.B3G_UP_ARROW in keys and (keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN):
            commands['pitch'] = self.pitch_val
        if p.B3G_DOWN_ARROW in keys and (keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN):
            commands['pitch'] = -self.pitch_val

        # --- Roll (Arrow Left/Right) ---
        if p.B3G_LEFT_ARROW in keys and (keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN):
            commands['roll'] = -self.roll_val
        if p.B3G_RIGHT_ARROW in keys and (keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN):
            commands['roll'] = self.roll_val

        return commands
