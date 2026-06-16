classdef Direction
    %DIRECTION  Motion command enumeration for all robot types.
    %   FORWARD/BACKWARD/LEFT/RIGHT   Translation in body x/y
    %   UP/DOWN                        Vertical motion (aerial robots)
    %   YAW_LEFT/YAW_RIGHT             Rotation about body z
    %   ROLL_LEFT/ROLL_RIGHT           Rotation about body x
    %   PITCH_UP/PITCH_DOWN            Rotation about body y
    %   STOP                           Hover/brake
    %   RESET                          Return to initial state
    enumeration
        FORWARD, BACKWARD, LEFT, RIGHT
        UP, DOWN
        YAW_LEFT, YAW_RIGHT
        ROLL_LEFT, ROLL_RIGHT
        PITCH_UP, PITCH_DOWN
        STOP, RESET
    end
end
