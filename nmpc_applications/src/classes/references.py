#!/usr/bin/python3.8

from classes.all_imports import *
from classes.common_class import *

sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

""" 
    Reference class

    Methods to define references
""" 

class Reference():
    def __init__(self, option=2, qa=1.0, qb=1.0, qc=1.0 ):

        #   Point
        s = ca.SX.sym('s')

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')
        
        #   Straight line on hill
        if( option == 0 ):
            self.x_ref = ca.Function( 'x_ref', [s], [s] )
            self.y_ref = ca.Function( 'y_ref', [s], [0.0] )
            self.yaw_ref = ca.Function( 'yaw_ref', [s], [0.0] )
            self.curvature = ca.Function( 'curvature', [s], [0.0] )

            #   Z = a * x
            _a = qa

            nx = -_a
            ny = 0.0
            nz = 1.0

            n = ca.vertcat(nx, ny, nz)
            t = ca.vertcat(ca.cos(yaw), ca.sin(yaw), 0.0)

            f = t - ca.dot(t, n) * n
            r = ca.cross(f, n)

            pitch = -ca.asin( -f[2] )
            roll = ca.asin( r[2] / ca.cos(pitch) )

            self.roll_ref = ca.Function( 'roll', [x, y, yaw], [roll] )
            self.pitch_ref = ca.Function( 'pitch', [x, y, yaw], [pitch] )

        elif( option == 1 ):
            #   Lissajous curve (a=3, delta=pi/2, b=2)
            A = 2
            B = 2
            a = 3
            delta = ca.pi/2
            b = 2
        
        elif( option == 2 ):
            #   Lissajous curve (a=1, delta=pi/2, b=2)
            A = 2
            B = 2
            a = 1
            delta = ca.pi/2
            b = 2

        if( option == 1 or option == 2 ):
            dx = A * a * ca.cos(a * s + delta)
            dy = B * b * ca.cos(b * s)

            ddx = -A * (a**2) * ca.sin(a * s + delta)
            ddy = -B * (b**2) * ca.sin(b * s)

            self.x_ref = ca.Function( 'x_ref', [s], [ A * ca.sin(a * s + delta) ] )
            self.y_ref = ca.Function( 'y_ref', [s], [ B * ca.sin(b * s) ] )
            self.yaw_ref = ca.Function( 'yaw_ref', [s], [ ca.atan2(dy, dx) ] )
            self.curvature = ca.Function( 'curvature', [s], [ (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5 ] )
            
            #   z = c * sin(a*x) * cos(b * y)
            _a = qa
            _b = qb
            _c = qc
            
            nx = -_c * _a * ca.cos(_a * x) * ca.cos(_b * y)
            ny = _c * _b * ca.sin(_a * x) * ca.cos(_b * y)
            nz = 1.0

            n = ca.vertcat(nx, ny, nz)
            t = ca.vertcat(ca.cos(yaw), ca.sin(yaw), 0.0)

            f = t - ca.dot(t, n) * n
            r = ca.cross(f, n)

            pitch = -ca.asin( -f[2] )
            roll = ca.asin( r[2] / ca.cos(pitch) )

            self.roll_ref = ca.Function( 'roll', [x, y, yaw], [roll] )
            self.pitch_ref = ca.Function( 'pitch', [x, y, yaw], [pitch] )

    def _getReference(self, progress, virtual_speed, sampling_time, horizon):
        
        #   Get reference
        p = progress

        x_r = []
        y_r = []
        yaw_r = []

        index = 0

        while( index <= horizon ):
            new_x_r = float( self.x_ref(p) )
            new_y_r = float( self.y_ref(p) )
            new_yaw_r = float( self.yaw_ref(p) )

            x_r += [ new_x_r ]
            y_r += [ new_y_r ]
            yaw_r += [ new_yaw_r ]

            p += virtual_speed * sampling_time

            index += 1
        ###

        return x_r, y_r, yaw_r

    def _getAdjustedReference(self, yaw, progress, virtual_speed, sampling_time, horizon):
        
        #   Get reference
        p = progress

        x_r = []
        y_r = []
        yaw_r = []

        index = 0

        while( index <= horizon ):
            
            new_x_r = float( self.x_ref(p) )
            new_y_r = float( self.y_ref(p) )

            if( index == 0 ):
                new_yaw_r = float( self.yaw_ref(p) )

                if( abs(new_yaw_r - yaw) > math.pi ):
                    
                    diff = new_yaw_r - yaw

                    if( diff > 0 ):
                        diff = 2 * math.pi - diff

                    elif( diff < 0 ):
                        diff = diff + 2 * math.pi

                    new_yaw_r = yaw + diff

                else:
                    pass

            elif( index > 0 ):
                prev_yaw_r = new_yaw_r
                new_yaw_r = float( self.yaw_ref(p) )

                if( abs(new_yaw_r - prev_yaw_r) > math.pi ):
                    
                    diff = new_yaw_r - prev_yaw_r

                    if( diff > 0 ):
                        diff = 2 * math.pi - diff

                    elif( diff < 0 ):
                        diff = diff + 2 * math.pi

                    new_yaw_r = prev_yaw_r + diff

                else:
                    pass
            
            x_r += [ new_x_r ]
            y_r += [ new_y_r ]
            yaw_r += [ new_yaw_r ]

            p += virtual_speed * sampling_time

            index += 1
        ###

        return x_r, y_r, yaw_r
    
    def _getLinearReference(self, x0, y0, virtual_speed, sampling_time, horizon, m):
        
        #   Get reference
        x_r = [x0]
        y_r = [y0]
        yaw_r = [ ca.tan(m) ]

        index = 0

        while( index < horizon ):
            x_r += [ x_r[index] + ca.cos(m) * virtual_speed * sampling_time ]
            y_r += [ y_r[index] + ca.sin(m) * virtual_speed * sampling_time ]
            yaw_r += [ ca.tan(m) ]

            index += 1
        ###

        return x_r, y_r, yaw_r

    def _combinedReference(self, x0, y0, progress, virtual_speed, sampling_time, horizon, m, flag):

        """
        Combine straight line reference with default reference. The goal is to guide the vehicle to the default reference
        in case it is far away from it.
        
        :param self: Description
        """

        x = x0
        y = y0

        x_r = [x]
        y_r = [y]
        yaw_r = [ ca.tan(m) ]

        error = ( ( x - self.x_ref(progress) )**2 + ( y - self.y_ref(progress) )**2 )**0.5
        direction = ca.atan2( y - self.y_ref(progress), x - self.x_ref(progress) )
        new_direction = direction

        index = 0

        p = progress

        if( error > 1e-1 and flag == True ):
            while(True):
                if( direction == new_direction and index < horizon ):
                    next_x_r = x_r[index] + ca.cos(m) * virtual_speed * sampling_time
                    next_y_r = y_r[index] + ca.sin(m) * virtual_speed * sampling_time
                    next_yaw_r = ca.tan(m)

                    new_direction = ca.atan2( next_y_r - self.y_ref(progress), next_x_r - self.x_ref(progress) )

                    x_r += [ next_x_r ]
                    y_r += [ next_y_r ]
                    yaw_r += [ next_yaw_r ]

                    index += 1

                elif( direction != new_direction and index < horizon ):
                    new_x_r = float( self.x_ref(p) )
                    new_y_r = float( self.y_ref(p) )
                    new_yaw_r = float( self.yaw_ref(p) )

                    x_r += [ new_x_r ]
                    y_r += [ new_y_r ]
                    yaw_r += [ new_yaw_r ]

                    p += virtual_speed * sampling_time

                    index += 1
                
                else:
                    break
        
        else:
            while( index < horizon ):
                new_x_r = float( self.x_ref(p) )
                new_y_r = float( self.y_ref(p) )
                new_yaw_r = float( self.yaw_ref(p) )

                x_r += [ new_x_r ]
                y_r += [ new_y_r ]
                yaw_r += [ new_yaw_r ]

                p += virtual_speed * sampling_time

                index += 1
        
        return x_r, y_r, yaw_r