import numpy as np
import math

def cubic_bezier(P0, P1, P2, P3, n=5):
    """
    Cubic bezier
    @params:
        P0       - Required  : Start point ((Float),(Float))
        P1       - Required  : Start control point ((Float),(Float))
        P2       - Required  : End control point ((Float),(Float))
        P3       - Required  : End point ((Float),(Float))
        n        - Optional  : Number of points on the curve (Int)
    """
    curve_x = []
    curve_y = []
    
    # B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
    for t in np.linspace(0, 1, n):
        P0_x = pow((1 - t), 3) * P0[0]
        P0_y = pow((1 - t), 3) * P0[1]

        P1_x = 3 * pow((1-t), 2) * t * P1[0]
        P1_y = 3 * pow((1-t), 2) * t * P1[1]

        P2_x = 3 * (1-t) * pow(t, 2) * P2[0]
        P2_y = 3 * (1-t) * pow(t, 2) * P2[1]

        P3_x = pow(t, 3) * P3[0]
        P3_y = pow(t, 3) * P3[1]

        curve_x.append(P0_x + P1_x + P2_x + P3_x)
        curve_y.append(P0_y + P1_y + P2_y + P3_y)
    return curve_x, curve_y

def calc_path_to_pit(pit_pos, own_pos, enemy_pos, own_yaw, pd=0.8, cd=0.4, pc=5):
    """
    Calculate path to pit
    @params:
        pit_pos     - Required  : Position of pit ((Float), (Float))
        own_pos     - Required  : Position of own robot ((Float), (Float))
        enemy_pos   - Required  : Position of enemy robot ((Float), (Float))
        pd          - Optional  : Distance of control points from robots (Float)
        cd          - Optional  : Distance from the mid_point for new control points (Float)
        pc          - Optional  : How many points on the curve * 2 (Int) 
    """
    c_points = 1 / pc
    
    enemy_angle_to_pit = math.atan2(enemy_pos[1]-pit_pos[1], enemy_pos[0]-pit_pos[0])
    
    # control point for the bezier curve. enemy will be between this and the pit.
    # x₂ = x₁ + cos A * hypotenuse
    # y₂ = y₁ + sin A * hypotenuse
    enemy_p_x = enemy_pos[0] + math.cos(enemy_angle_to_pit) * pd
    enemy_p_y = enemy_pos[1] + math.sin(enemy_angle_to_pit) * pd
    enemy_p = (enemy_p_x, enemy_p_y)
    
    
    # x₂ = x₁ + cos A * hypotenuse
    # y₂ = y₁ + sin A * hypotenuse
    # control point for the bezier curve. facing away from front of our own robot.
    own_p_x = own_pos[0] + math.cos(own_yaw) * pd
    own_p_y = own_pos[1] + math.sin(own_yaw) * pd
    own_p = (own_p_x, own_p_y)
    
    # x₃ = (x₁+x₂)/2
    # y₃ = (y₁+y₂)/2
    # mid point between the 2 control points
    mid_point_x = (own_p[0]+enemy_p[0])/2
    mid_point_y = (own_p[1]+enemy_p[1])/2
    mid_point = (mid_point_x, mid_point_y)
    
    # d = √((x₂-x₁)²+(y₂-y₁)²)
    # distance from mid_point to control points
    distance_mp_enemy = math.sqrt(pow(enemy_p_x-mid_point_x,2) + pow(enemy_p_y-mid_point_y,2)) 
    distance_mp_own = math.sqrt(pow(own_p_x-mid_point_x,2) + pow(own_p_y-mid_point_y,2))
    
    # x₃ = x₁+d/D(x₂−x₁)
    # y₃ = y₁+d/D(y₂−y₁)
    # D = Distance between (x₁,y₁) and (x₂,y₂)
    # d = distance our point will be in
    # mid control point for enemy 
    mp_enemy1_x = mid_point_x + cd / distance_mp_enemy * (enemy_p_x-mid_point_x)
    mp_enemy1_y = mid_point_y + cd / distance_mp_enemy * (enemy_p_y-mid_point_y)
    mp_enemy1 = (mp_enemy1_x, mp_enemy1_y)
    
    # x₃ = x₁+d/D(x₂−x₁)
    # y₃ = y₁+d/D(y₂−y₁)
    # D = Distance between (x₁,y₁) and (x₂,y₂)
    # d = distance our point will be in
    # mid control point for own robot
    mp_own1_x = mid_point_x + cd / distance_mp_own * (own_p_x-mid_point_x)
    mp_own1_y = mid_point_y + cd / distance_mp_own * (own_p_y-mid_point_y)
    mp_own1 = (mp_own1_x, mp_own1_y)
    
    # x₃ = (x₁+x₂)/2
    # y₃ = (y₁+y₂)/2
    # mid point between enemy_p and enemy_pos
    mp_enemy2_x = (enemy_p[0]+enemy_pos[0])/2
    mp_enemy2_y = (enemy_p[1]+enemy_pos[1])/2
    mp_enemy2 = (mp_enemy2_x, mp_enemy2_y)
    
    # x₃ = (x₁+x₂)/2
    # y₃ = (y₁+y₂)/2
    # mid point between own_p and own_pos
    mp_own2_x = (own_pos[0]+own_p[0])/2
    mp_own2_y = (own_pos[1]+own_p[1])/2
    mp_own2 = (mp_own2_x, mp_own2_y)
    
    # Add everything together
    curve_x, curve_y = cubic_bezier(own_pos, mp_own2, mp_own1, mid_point, pc)
    curve2_x, curve2_y = cubic_bezier(mid_point, mp_enemy1, mp_enemy2, enemy_pos, pc)
    curve_x += curve2_x + [pit_pos[0]]
    curve_y += curve2_y + [pit_pos[1]]
    
    return curve_x, curve_y


def ros_to_radians(angle):
    """ Converts from range(-pi, pi) to range(0, 2pi)"""
    return angle % (2.0*math.pi)

def radians_to_ros(angle):
    """ Converts from range(0, 2pi) to range(-pi, pi)"""
    a = ros_to_radians(angle)
    if a > math.pi:
        a -= 2.0*math.pi
    return a