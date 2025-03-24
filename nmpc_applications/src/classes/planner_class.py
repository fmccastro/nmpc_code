#!/usr/bin/python3.8

from classes.common_class import *

class Planner(Common):

    metersHeightmap = None
    pixHeightmap = None

    def __init__(self):

        """
            Fast marching points
        """

        super().__init__()

        #   Initialize transformation parameters
        self.m_i2y = 0
        self.b_i2y = 0

        self.m_j2x = 0
        self.b_j2x = 0

        self.m_y2i = 0
        self.b_y2i = 0

        self.m_x2j = 0
        self.b_x2j = 0

    def _createElevationMap(self, fromOrigin):
        
        print("*** Create elevation map ***")
        self._loadHeightmap(fromOrigin)

        #plt.matshow(self.metersHeightmap)
        #plt.show()

        if(fromOrigin):
            self._metersHeight()
            
            print("Set location where map will be saved both on .dat and .npy formats.")
            _folder = self.mapFolderDir + str(self.mapFolder)
            mapNpySaveDir = _folder + "/" + "elevationMap" + ".npy"
            mapDataSaveDir = _folder + "/" + "elevationMap" + ".dat"

            pubDir = self.exportPlot2PubDir + "elevationMap" + str(self.mapFolder) + ".png"

            self._saveMap(mapNpySaveDir, mapDataSaveDir, self.metersHeightmap)
            self._export2Publish(self.metersHeightmap, pubDir, "height [m]")
            
            #self._plotHeightmap(self.metersHeightmap)

            print("\n")

    def _loadHeightmap(self, fromOrigin):
        
        """
            Load heightmap file from raw digital elevation model and convert it into .npy format
        """
        
        print("*** Load elevation map ***")

        if( fromOrigin ):
            print("Load elevation map from original file.")

            #   From .jpg or .png format into .npy format
            if( self.mapFile.endswith( ('.jpg', '.png') ) ):
                img = mpimg.imread(self.mapFile)

            #   From .tif format into .npy format
            elif( self.mapFile.endswith('.tif') ):
                #   Read original file
                img = Image.open(self.mapFile)

                #   Convert file to numpy format
                img = np.array(img)

            #   From .dem format into .npy format
            elif( self.mapFile.endswith('.dem') ):
                img = np.fromfile(self.mapFile)
            
            self.pixHeightmap = img

        else:
            print("Load elevation map from .npy file with meters units.")
            _folder = self.mapFolderDir + str(self.mapFolder)
            mapNpySaveDir = _folder + "/" + "elevationMap" + ".npy"

            img = np.load(mapNpySaveDir)

            self.metersHeightmap = img
        
        print("\n")

        return img

    def _metersHeight(self):

        """
            Convert height to meters and set minimum height to zero

            :heightmap (in numpy format)
        """

        print("***  Convert elevation map height to meters and set minimum height to zero.  ***")

        #   Get minimum height 
        z_min = self.pixHeightmap.min()

        #   Shift minimum height to zero
        pix_heightmap = self.pixHeightmap - z_min

        #   Convert height to meters
        self.metersHeightmap = pix_heightmap * self.mapHeight / pix_heightmap.max()

        print("\n")

        return self.metersHeightmap

    def _loadCostMap(self, option1, option3, radius = 0, threshold = 0):

        print("Set location where map will be loaded from (on .npy format).")
        _folder = self.mapFolderDir + str(self.mapFolder)

        if( option1 == "maxTerrainRoughness" ):
            costMapDir = _folder + "/" + option1 + ".npy"
        
        elif( option1 == "mapRefinement" ):
            costMapDir = _folder + "/" + option1 + "+" + option3 + "+" + str(threshold) + "+" + str(radius) + ".npy"

        else:
            costMapDir = _folder + "/" + option1 + "+" + option3 + ".npy"

        return np.load(costMapDir)

    def _saveMap(self, fileLocnpy, fileLocdat, map):

        """
            Save map into .npy and .csv formats
        """

        print("***  On function _saveMap  ***")

        print("Check if file exists.")
        if os.path.isfile(fileLocnpy):
            
            print("File exists. File was removed.")
            os.remove(fileLocnpy)
        
        else:
            print("File does not exist.")

        print("Save map expressed in meters units on .npy format.")
        np.save(fileLocnpy, map)

        ###################################################################
        print("\n")

        print("Check if file exists.")
        if os.path.isfile(fileLocdat):
            
            print("File exists. File was removed.")
            os.remove(fileLocdat)
        
        else:
            print("File does not exist.")

        print("Save map expressed in meters units on .dat format.")
        self._writeMap2File(map, fileLocdat)

        print("\n")
    
    def _getVirtualAxis(self, map):

        #   X is a vertical axis from 0 (top) to mapDimx (down)
        #   Y is a horizontal axis from 0 (left) to mapDimy (right)
        
        x = np.linspace( 0.0, self.mapDimx, map.shape[0] )
        y = np.linspace( 0.0, self.mapDimy, map.shape[1] )

        return x, y
    
    def _getVirtualMeshgrid(self, map):

        #   X is a vertical axis from 0 (top) to mapDimx (down)
        #   Y is a horizontal axis from 0 (left) to mapDimy (right)

        x = np.linspace( 0.0, self.mapDimx, map.shape[0] )
        y = np.linspace( 0.0, self.mapDimy, map.shape[1] )

        xx , yy = np.meshgrid(x, y)

        return xx, yy

    def _getRealMeshgrid(self, map):
        
        x = np.linspace( -self.mapDimx/2, self.mapDimx/2, map.shape[0] )
        y = np.linspace( self.mapDimy/2, -self.mapDimy/2, map.shape[1] )

        xx , yy = np.meshgrid(x, y)

        return xx, yy

    def _writeMap2File(self, map, fileLoc):
        
        print("*** On function _writeHeightmap2File ***")
        x, y = self._getVirtualAxis(map)

        print("Open file. Write to file then.")
        file = open(fileLoc, "w", newline="")

        for column in range(map.shape[0]):
            for row in range(map.shape[1]):

                #   Write to file. File is organized with three columns (x y height)
                file.write( str( round(x[column], 3) ) + " " + str( round( y[row], 3) ) + " " + str( round( map[column, row], 3) ) + "\n" )

        file.close()

        print("\n")

    def _plotMap(self, file2Plot, verLabel, includePath = None):
        
        matplotlib.rcParams.update({
            'font.family': 'sans-serif',
            'font.sans-serif': ["Helvetica"]
        })

        xx, yy = self._getRealMeshgrid(file2Plot)

        z_max = file2Plot.max()
        z_min = file2Plot.min()

        norm = matplotlib.colors.Normalize(vmin = z_min, vmax = z_max)

        fig, ax1 = plt.subplots(layout="constrained")
        m = cm.ScalarMappable(cmap = cm.YlOrBr)
        m.set_array(file2Plot)
        ax1.contourf(xx, yy, file2Plot, 100, cmap = cm.YlOrBr, norm = norm)
        clb = fig.colorbar(m, ax = ax1, label=verLabel)
        ax1.set_xlabel("x [m]", fontname="Arial")
        ax1.set_ylabel("y [m]", fontname="Arial")
        ax1.set_xticklabels(ax1.get_xticks(), fontname='Arial', fontsize=12)
        ax1.set_yticklabels(ax1.get_yticks(), fontname='Arial', fontsize=12)

        if( verLabel == "Height [m]" ):
            if(includePath is not None):
                ax1.scatter(self.goalPoint[0], self.goalPoint[1], s=20.0, c='r', marker='o')

                index = 0
                for path in includePath:
                    if( isinstance(path, np.ndarray) == True ):
                        if( index == 0 ):
                            ax1.scatter(path[0, 0], path[0, 1], s= 20.0, c='g', marker='o')
                        
                        if( index == 0 ):
                            line1, = ax1.plot( path[:, 0], path[:, 1], 'b-', label = 'mapRefinement+Surface' )
                    
                        elif( index == 1 ):
                            line2, = ax1.plot( path[:, 0], path[:, 1], 'b--', label = 'mapRefinement+Points' )
                        
                        elif( index == 2 ):
                            line3, = ax1.plot( path[:, 0], path[:, 1], 'r-', label = 'worstCase+Surface' )
                        
                        elif( index == 3 ):
                            line4, = ax1.plot( path[:, 0], path[:, 1], 'r--', label = 'worstCase+Points' )

                    index += 1
                
                ax1.legend(handles = [line1, line2, line3, line4])

        else:
            if(includePath is not None):
                ax1.scatter(self.goalPoint[0], self.goalPoint[1], s=10.0, c='r', marker='o')

                for path in includePath:
                    if( isinstance(path, np.ndarray) == True ):
                        ax1.scatter(path[0, 0], path[0, 1], c='g', marker='x')
                        ax1.plot( path[:, 0], path[:, 1], 'b' )

        #clb.ax.set_ylabel('height [m]', rotation=270, labelpad = 6.0)
        plt.show()

    def _export2Publish(self, map, saveDir, verLabel, includePath = None):

        print("*** On function _export2Publish ***")

        matplotlib.rcParams.update({
            'font.family': 'sans-serif',
            'font.sans-serif': ["Helvetica"]
        })

        xx, yy = self._getRealMeshgrid(map)

        z_max = map.max()
        z_min = map.min()

        norm = matplotlib.colors.Normalize( vmin = z_min, vmax = z_max )
    
        fig, ax1 = plt.subplots(layout="constrained")
        m = cm.ScalarMappable(cmap = cm.YlOrBr)
        m.set_array(map)
        ax1.contourf(xx, yy, map, 50, cmap = cm.YlOrBr, norm = norm)
        clb = fig.colorbar(m, ax = ax1, label=verLabel)
        ax1.set_xlabel("x [m]", fontname="Arial")
        ax1.set_ylabel("y [m]", fontname="Arial")
        ax1.set_xticklabels(ax1.get_xticks(), fontname='Arial', fontsize=12)
        ax1.set_yticklabels(ax1.get_yticks(), fontname='Arial', fontsize=12)
        #clb.ax.set_ylabel('height [m]', rotation=270, labelpad = 6.0)
        #plt.rcParams["font.family"] = "sans-serif"
        #plt.rcParams["font.sans-serif"] = "Helvetica"

        if( verLabel == "Height [m]" ):
            if(includePath is not None):
                ax1.scatter(self.goalPoint[0], self.goalPoint[1], s=20.0, c='r', marker='o')

                index = 0
                for path in includePath:
                    if( isinstance(path, np.ndarray) == True ):
                        if( index == 0 ):
                            ax1.scatter(path[0, 0], path[0, 1], s= 20.0, c='g', marker='o')
                        
                        if( index == 0 ):
                            line1, = ax1.plot( path[:, 0], path[:, 1], 'b-', label = 'worstCase+Points' )
                    
                        elif( index == 1 ):
                            line2, = ax1.plot( path[:, 0], path[:, 1], 'b--', label = 'worstCase+Surface' )
                        
                        elif( index == 2 ):
                            line3, = ax1.plot( path[:, 0], path[:, 1], 'r-', label = 'mapRefinement+Points' )
                        
                        elif( index == 3 ):
                            line4, = ax1.plot( path[:, 0], path[:, 1], 'r--', label = 'mapRefinement+Surface' )

                    index += 1
                
                #   Place legend and set font size
                ax1.legend(handles = [line1, line2, line3, line4], loc=2, fontsize = 10)

        else:
            if(includePath is not None):
                ax1.scatter(self.goalPoint[0], self.goalPoint[1], s=10.0, c='r', marker='o')

                for path in includePath:
                    if( isinstance(path, np.ndarray) == True ):
                        ax1.scatter(path[0, 0], path[0, 1], c='g', marker='x')
                        ax1.plot( path[:, 0], path[:, 1], 'b' )

        print("Check if file exists.")
        if os.path.isfile(saveDir):
            
            print("File exists. File was removed.")
            os.remove(saveDir)
        
        else:
            print("File does not exist.")

        print("Save new file.")
        plt.savefig(saveDir, bbox_inches='tight',transparent=True)

        print("\n")
    
    def _updateTransformationParameters(self, map):

        """
            Pixels to meters transformation (real frame)

            (0, 25) -----------------------------
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
        (500, -25)  -----------------------------
                (0, -25)                    (500, 25)
        """

        map_shapei = map.shape[0] - 1
        map_shapej = map.shape[1] - 1

        self.m_i2y = (-self.mapDimy/2 - self.mapDimy/2) / map_shapei
        self.b_i2y = self.mapDimy/2

        self.m_j2x = (self.mapDimx/2 - (-self.mapDimx/2)) / map_shapej
        self.b_j2x = -self.mapDimy/2

        """
            Meters to pixels transformation (from real frame)

            (25, 0) |---------------------------|
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
                    |                           |
        (-25, 500)  |---------------------------|
                (-25, 0)                    (25, 500)
        """

        self.m_y2i = map_shapei / (-self.mapDimy)
        self.b_y2i = -self.m_y2i * self.mapDimy / 2

        self.m_x2j = map_shapej / self.mapDimx
        self.b_x2j = -self.m_x2j * (-self.mapDimx / 2)

    def _transformation(self, x, y, option = "pix2MetersRealFrame"):

        if( option == "pix2MetersRealFrame" ):
            output_x = self.m_j2x * y + self.b_j2x
            output_y = self.m_i2y * x + self.b_i2y
            return output_x, output_y
    
        elif( option == "meters2PixRealFrame" ):
            output_x = int(self.m_y2i * y + self.b_y2i)
            output_y = int(self.m_x2j * x + self.b_x2j)
            return output_x, output_y

    def _frame_pix2Meters(self, x, y, heightmap):

        """
            Convert pixels to meters and transform raw frame into a map-centered gazebo world frame

            :x x coordinate matrix
            :y y coordinate matrix
            :heightmap elevation grid matrix
        """

        mx = self.mapDimx / ( heightmap.shape[0] - 1 )
        my = self.mapDimy / ( heightmap.shape[1] - 1 )

        bx = 0.0
        by = 0.0

        x_mt = x * mx + bx
        y_mt = y * my + by

        return x_mt, y_mt

    def _frame_meters2Pix(self, x, y):

        """
            Convert point on meters to pix
        """

        #   Virtual x axis: from -mapDimx/2 to mapDimx/2
        #   Virtual y axis: from -mapDimy/2 to mapDimy/2 

        pixDimx = self.metersHeightmap.shape[0] - 1
        pixDimy = self.metersHeightmap.shape[1] - 1

        mx = pixDimx / self.mapDimx
        bx = 0

        my = pixDimy / self.mapDimy
        by = 0

        x_pix = mx * x + bx
        y_pix = my * y + by

        x_pix_red = int(x_pix)
        y_pix_red = int(y_pix)

        if(x_pix_red > pixDimx):
            x_pix_red = pixDimx
        
        elif( x_pix_red < 0 ):
            x_pix_red = 0
        
        if(y_pix_red > pixDimy):
            y_pix_red = pixDimy

        elif(y_pix_red < 0):
            y_pix_red = 0

        return x_pix_red, y_pix_red

    def _real2VirtualFrame(self, x_r, y_r):

        mx = self.mapDimx / (-self.mapDimx/2 - self.mapDimx/2)
        bx = -mx * self.mapDimx / 2

        my = self.mapDimy / (self.mapDimy/2 - (-self.mapDimy/2))
        by = -my * (-self.mapDimy / 2)

        x_v = mx * x_r + bx
        y_v = my * y_r + by

        return x_v, y_v

    def _virtual2RealFrame(self, x_v, y_v):

        mx = -self.mapDimx / self.mapDimx
        bx = self.mapDimx/2

        my = self.mapDimx / self.mapDimx
        by = -self.mapDimx / 2

        x_r = mx * x_v + bx
        y_r = my * y_v + by
        
        return x_r, y_r

    def _meters2Pix(self, x, y):

        """
            Convert meters to pixels

            :x dimension on x direction of world frame
            :y dimension on y direction of world frame

            return each dimension in pixel dimension
        """
        
        x_pix = x * self.metersHeightmap.shape[0] / self.mapDimx
        y_pix = y * self.metersHeightmap.shape[1] / self.mapDimy

        return int(x_pix), int(y_pix)
    
    def _pix2Meters(self):

        """
            Convert index to meters

            :x dimension on x direction of world frame
            :y dimension on y direction of world frame

            return each dimension in meters
        """

        print("***  On function _pix2Meters ***")
        
        print("Create array of divisons of elevation map.")
        x = np.arange(0, self.metersHeightmap.shape[0], 1)
        y = np.arange(0, self.metersHeightmap.shape[1], 1)

        print("Convert indexed divison by cells to meters unit.")
        x_met = x / self.metersHeightmap.shape[0] * self.mapDimx
        y_met = y / self.metersHeightmap.shape[1] * self.mapDimy

        print("\n")

        return x_met, y_met

    def _getMapFrame(self, heightmap, option = True):
        
        """
            Define map frame

            :heightmap heightmap in numpy format
            :option whether to convert heights to meters or not
        """

        #   Define cell index array for x and y world frame directions
        x = np.arange( 0, int( heightmap.shape[0] ), 1 )
        y = np.arange( 0, int( heightmap.shape[1] ), 1 )
        
        #   Create 2-D grid from 1-D arrays
        X, Y = np.meshgrid(x, y)

        #   Map needs to be viewed downwards
        if( heightmap.ndim == 2 ):
            Z = heightmap[np.flip(Y, 0), X]

        elif( heightmap.ndim == 3 ):
            Z = heightmap[np.flip(Y, 0), X, 0]

        #   Transform raw elevation grid frame into gazebo world frame
        X_new, Y_new = self._frame_pix2Meters(X, Y, Z)

        #   Convert height to meters
        if( option ):
            Z_new = self._metersHeight()

        else:
            Z_new = heightmap

        return X_new, Y_new, Z_new

    def _continuousHeightmap_casadi(self):
        
        """
            Create continuous elevation map from interpolation of discretized one

            Return:
                    :x_mt   Divisons of x dimension in meters units
                    :y_mt   Divisions of y dimenson in meters units
                    :continuousHeightmap    continuous elevation map
        """ 
        
        print("***  On function _continuousHeightmap_casadi ***")

        #   Build fake strictly increasing arrays to feed interpolant function

        x_mt, y_mt = self._getVirtualAxis(self.metersHeightmap)

        data = self.metersHeightmap.ravel(order = 'F')

        print("Interpolate discrete map to yield a continuous heightmap")
        continuousHeightmap = ca.interpolant('continuousHeightmap', 'bspline', [x_mt, y_mt], data)

        print("\n")

        return x_mt, y_mt, continuousHeightmap
    
    def _heightmapGradient(self, continuousElevationMap):
        
        print("***  on function _heightmapGradient  ***")
        print("Compute gradient of elevation map")
        x = ca.MX.sym('x', 2)

        map_jac = ca.Function('map_jac', [x], [ ca.jacobian(continuousElevationMap(x), x) ] )

        print("\n")

        return map_jac

    def _function2MinimizeCasadi(self, continuousHeightmap, option3, thetaDiv, yDiv):
        
        """
            Define function to minimize and constraints
            
            :continuousHeightmap interpolated elevation grid from discrete one
            :x cell for index 0
            :y cell for index 1
            :yaw robot fixed yaw
            :option3    Points  ;   Surface
        """

        print("***  on _function2MinimizeCasadi    ***")

        print("Define optimization problem with constraints for each yaw and each cell.")

        #   Define wheel contact points with respect to world frame (frame origin defined at left upper corner; to be changed later)

        #   State that every defined point height cannot be lower than the map's height at the respective cell
        #   State that the modulus of roll and modulus of pitch cannot be higher than roll_max and pitch_max
        #   x_opt[0] : z , x_opt[1] : roll , x_opt[2] : pitch
        x = ca.MX.sym('x')
        y = ca.MX.sym('y')
        z = ca.MX.sym('z')
        roll = ca.MX.sym('roll')
        pitch = ca.MX.sym('pitch')
        yaw = ca.MX.sym('yaw')

        #   Left back wheel contact point
        wheel_lb_x, wheel_lb_y, wheel_lb_z = self._rotation3DBody2World(roll, pitch, yaw, -self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, z)
        ###

        #   Left front wheel contact point
        wheel_lf_x, wheel_lf_y, wheel_lf_z = self._rotation3DBody2World(roll, pitch, yaw, self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, z)
        ###

        #   Right back wheel contact point
        wheel_rb_x, wheel_rb_y, wheel_rb_z = self._rotation3DBody2World(roll, pitch, yaw, -self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, z)
        ###

        #   Right front wheel contact point
        wheel_rf_x, wheel_rf_y, wheel_rf_z = self._rotation3DBody2World(roll, pitch, yaw, self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, z)
        ###

        if( option3 == "Points" ):
            f = z

            g = ca.vertcat( z - continuousHeightmap( ca.vertcat(x, y) ),\
                            wheel_lb_z - continuousHeightmap( ca.vertcat(wheel_lb_x, wheel_lb_y) ),\
                            wheel_lf_z - continuousHeightmap( ca.vertcat(wheel_lf_x, wheel_lf_y) ),\
                            wheel_rb_z - continuousHeightmap( ca.vertcat(wheel_rb_x, wheel_rb_y) ),\
                            wheel_rf_z - continuousHeightmap( ca.vertcat(wheel_rf_x, wheel_rf_y) ) )
            
            w = ca.vertcat(z, roll, pitch)
            p = ca.vertcat(x, y, yaw)
            
            prob = {'f': f, 'x': w, 'g': g, 'p': p}

            solver = ca.nlpsol('solver', self.optSolver, prob, self.optOptions)
        
        elif( option3 == "Surface" ):
            
            #   Left back wheel points
            g_con_lb = self._wheelSurfacePoints(-self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, 0.0, x, y, z, roll, pitch, yaw, continuousHeightmap, thetaDiv, yDiv)

            #   Left front wheel points
            g_con_lf = self._wheelSurfacePoints(self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, 0.0, x, y, z, roll, pitch, yaw, continuousHeightmap, thetaDiv, yDiv)
            
            #   Right back wheel points
            g_con_rb = self._wheelSurfacePoints(-self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, 0.0, x, y, z, roll, pitch, yaw, continuousHeightmap, thetaDiv, yDiv)

            #   Right front wheel points
            g_con_rf = self._wheelSurfacePoints(self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, 0.0, x, y, z, roll, pitch, yaw, continuousHeightmap, thetaDiv, yDiv)

            f = z

            w = ca.vertcat(z, roll, pitch)
            g = ca.vertcat(z - continuousHeightmap( ca.vertcat(x, y) ),\
                            g_con_lb,\
                            g_con_lf,\
                            g_con_rb,\
                            g_con_rf)
            p = ca.vertcat(x, y, yaw)

            prob = {'f': f, 'x': w, 'g': g, 'p': p}
            
            solver = ca.nlpsol('solver', self.optSolver, prob, self.optOptions)
            
        print("\n")

        return solver
    
    def _rotation3DBody2World(self, roll, pitch, yaw, xb, yb, zb, x, y, z):

        """
            Robot orientation
            :roll
            :pitch
            :yaw

            Wheel position in body frame
            :xb
            :yb
            :zb

            Robot position in world frame
            :x
            :y
            :z
        """

        #print("***  on function _rotation3DBody2World   ***")

        #print("Transform wheel position from body frame to world frame.")

        wheel_x = x + ca.cos(pitch) * ca.cos(yaw) * xb \
                        + ( ca.sin(roll) * ca.sin(pitch) * ca.cos(yaw) - ca.cos(roll) * ca.sin(yaw) ) * yb \
                        + ( ca.cos(roll) * ca.sin(pitch) * ca.cos(yaw) + ca.sin(roll) * ca.sin(yaw) ) * zb
        
        wheel_y = y + ca.cos(pitch) * ca.sin(yaw) * xb \
                        + ( ca.sin(roll) * ca.sin(pitch) * ca.sin(yaw) + ca.cos(roll) * ca.cos(yaw) ) * yb \
                        + ( ca.cos(roll) * ca.sin(pitch) * ca.sin(yaw) - ca.sin(roll) * ca.cos(yaw) ) * zb 
        
        wheel_z = z - ca.sin(pitch) * xb + ca.sin(roll) * ca.cos(pitch) * yb + ca.cos(roll) * ca.cos(pitch) * zb
    
        #print("\n")

        return wheel_x, wheel_y, wheel_z

    def _wheelSurfacePoints(self, x0, y0, z0, x, y, z, roll, pitch, yaw, continuousHeightmap, thetaDiv, yDiv):

        """
            Define wheel surface points symbolically

            Input wheel center point with respect to frame B
            :x0
            :y0
            :z0

            Robot position in world frame
            :x
            :y
            :z

            Robot orientation
            :roll
            :pitch
            :yaw
        """

        distances = []

        for angDiv in np.linspace(-3* math.pi / 4, -math.pi/4, thetaDiv):
            if( yDiv == 1 ):
                latDiv = 0

                x_wheel_b = x0 + self.wheelRadius * math.cos(angDiv)
                y_wheel_b = y0 + latDiv
                z_wheel_b = z0 + self.wheelRadius * math.sin(angDiv)

                x_wheel_w, y_wheel_w, z_wheel_w = self._rotation3DBody2World(roll, pitch, yaw, x_wheel_b, y_wheel_b, z_wheel_b, x, y, z)

                vertDist2Ground = z_wheel_w - continuousHeightmap( ca.vertcat(x_wheel_w, y_wheel_w) )

                distances += [vertDist2Ground]
            
            elif( yDiv > 1 ):
                for latDiv in np.linspace(-self.wheelWidth/2, self.wheelWidth/2, yDiv):
                    x_wheel_b = x0 + self.wheelRadius * math.cos(angDiv)
                    y_wheel_b = y0 + latDiv
                    z_wheel_b = z0 + self.wheelRadius * math.sin(angDiv)

                    x_wheel_w, y_wheel_w, z_wheel_w = self._rotation3DBody2World(roll, pitch, yaw, x_wheel_b, y_wheel_b, z_wheel_b, x, y, z)

                    vertDist2Ground = z_wheel_w - continuousHeightmap( ca.vertcat(x_wheel_w, y_wheel_w) )

                    distances += [vertDist2Ground]
        
        return ca.vertcat(*distances)
    
    def _terrainTraversability(self, option1, option2, option3, yaw_def, thetaDiv = 2, yDiv = 2):
        
        """
            Compute terrain inclination at each cell by solving an optimization problem. To meet that end, a continuous elevation map is obtained by interpolation of the discrete map.
            
            :option1    HeightMin           ;   MaxTerrainRoughness     ;       HeightMin_maxInclination
            :option2    NO_InitialGuess     ;   With_InitialGuess_X0
            :option3    Points              ;   Surface    

            Surface only options

            :thetaDiv
            :yDiv                                                
        """

        print("***  On function _terrainTraversability ***")

        print("Set dimensions of terrain traversability map.")
        terrainTraversabilityMap = np.zeros( ( self.metersHeightmap.shape[0], self.metersHeightmap.shape[1] ) )

        x, y, cont_heightmap = self._continuousHeightmap_casadi()

        #   Generate elevation gradient for the sake of results discussion
        gradMap = self._heightmapGradient(cont_heightmap)

        """#   Check elevation gradient positioning
        elevationMapGradient = np.zeros( ( self.metersHeightmap.shape[0], self.metersHeightmap.shape[1] ) )
        id = 0

        for i in x:
        
            jd = 0

            for j in y:
                grad = cont_heightmap( [i, j] )

                #elevationMapGradient[id, jd] = np.linalg.norm( np.array( [ grad[0, 0], grad[0, 1] ] ) )
                elevationMapGradient[id, jd] = grad

                jd += 1
            
            id += 1
        
        plt.close()
        plt.matshow(elevationMapGradient)
        plt.show()"""

        solver = self._function2MinimizeCasadi( cont_heightmap, option3, thetaDiv, yDiv )

        results_dict = {}

        n_opt = 0

        id = 0

        print("Start filling traversability map.")
        start = time.time()

        for i in x:
            
            print("Column: ", len(x) - id)

            jd = 0

            for j in y:

                #   Define circled area around which robot will turn
                top, bottom, right, left = self._limitsCircle(i, j)

                #   In case some area is outside the map, define the terrain inclination as pi/2 (max inclination possible)
                if( top < x[0] or bottom > x[-1] or right > y[-1] or left < y[0] ):
                    if( option1 == "HeightMin" or option1 == "HeightMin_maxInclination" ):
                        terrainTraversabilityMap[id, jd] = math.pi/2

                    if( option1 == "maxTerrainRoughness" ):
                        terrainTraversabilityMap[id, jd] = None

                #   Define inclination at the respective cell as an average of the optimized inclinations at different yaws
                else:
                    if( option1 == "HeightMin" ):
                        
                        if( option2 == "NO_InitialGuess" ):
                            x0 = [self.robotHeight * 2 + self.mapHeight, 0, 0]
                            p = [i, j, yaw_def]

                        elif( option2 == "With_InitialGuess_X0" ):
                            x0 = self._setInitialGuess(i, j, yaw_def, cont_heightmap)
                            p = [i, j, yaw_def]
                        
                        if( option3 == "Points" ):
                            sol = solver( x0 = x0, p = p,\
                                          lbx = [0, -math.pi/4, -math.pi/4], ubx = [ca.inf, math.pi/4, math.pi/4],\
                                          lbg = [0.0] * 5, ubg = [ca.inf] * 5 )
                        
                        elif( option3 == "Surface" ):
                            sol = solver( x0 = x0, p = p,\
                                          lbx = [0, -math.pi/4, -math.pi/4], ubx = [ca.inf, math.pi/4, math.pi/4],\
                                          lbg = [0.0] * (thetaDiv * yDiv * 4 + 1), ubg = [ca.inf] * (thetaDiv * yDiv * 4 + 1) )

                        g_opt = sol["g"].elements()

                        #   Collect the distance of each hull point to the ground (vertically)
                        if( option3 == "Points" ):
                            contacts = 0

                            for element in g_opt[1:]:
                                if( element <= self.wheelRadius * math.pow(10, -1) ):
                                    contacts += 1
                        
                        elif( option3 == "Surface" ):
                            contacts = 0

                            elements = [ min(g_opt[1: thetaDiv * yDiv + 1  ]), min(g_opt[ thetaDiv * yDiv + 1 : thetaDiv * yDiv * 2 + 1 ]),\
                                         min(g_opt[ thetaDiv * yDiv * 2 + 1 : thetaDiv * yDiv * 3 + 1  ]), min(g_opt[ thetaDiv * yDiv * 3 + 1 :]) ]

                            #print(elements)

                            for element in elements:
                                if( element <= self.wheelRadius * math.pow(10, -1) ):
                                    contacts += 1

                        #   Check if each wheel touches the ground
                        if( contacts >= 3 ):
                            x_opt = sol["x"].elements()

                            _roll_opt = x_opt[1]
                            _pitch_opt = x_opt[2]
                            
                            #   Retrieve inclination
                            inclination = self._inclination(_roll_opt, _pitch_opt)
                            terrainTraversabilityMap[id, jd] = inclination

                        else:
                            inclination = math.pi/2
                            terrainTraversabilityMap[id, jd] = inclination

                        fullSol = { "f": sol["f"].elements(), "g": sol["g"].elements(),\
                                    "lam_g": sol["lam_g"].elements(), "lam_p": sol["lam_p"].elements(), "lam_x": sol["lam_x"].elements(),\
                                    "x": sol["x"].elements() }

                        results_dict[str(n_opt)] = {"stats": solver.stats(), "solution": fullSol, 'inclination': inclination, 'contacts': contacts, 'gradient': gradMap( [i, j] ).elements() }

                    elif( option1 == "HeightMin_maxInclination" ):

                        list_inclinations = []

                        results_dict[ str(n_opt) ] = []

                        local_dict = {}

                        for yaw in np.arange(-math.pi, math.pi, 0.5):

                            if( option2 == "NO_InitialGuess" ):
                                x0 = [self.robotHeight * 2 + self.mapHeight, 0, 0, 0]
                                p = [i, j, yaw]

                            elif( option2 == "With_InitialGuess_X0" ):
                                x0 = self._setInitialGuess(i, j, yaw, cont_heightmap)
                                p = [i, j, yaw]
                            
                            if( option3 == "Points" ):
                                sol = solver( x0 = x0, p = p,\
                                                lbx = [0, -math.pi/4, -math.pi/4], ubx = [ca.inf, math.pi/4, math.pi/4],\
                                                lbg = [0.0] * 5, ubg = [ca.inf] * 5 )
                                                                                    
                            elif(option3 == "Surface" ):
                                sol = solver( x0 = x0, p = p,\
                                                lbx = [0, -math.pi/4, -math.pi/4], ubx = [ca.inf, math.pi/4, math.pi/4],\
                                                lbg = [0.0] * (thetaDiv * yDiv * 4 + 1), ubg = [ca.inf] * (thetaDiv * yDiv * 4 + 1) )

                            #   Collect the distance of each hull point to the ground (vertically)
                            g_opt = sol["g"].elements()

                            if( option3 == "Points" ):
                                contacts = 0

                                for element in g_opt[1:]:
                                    if( element <= self.wheelRadius * math.pow(10, -1) ):
                                        contacts += 1
                            
                            elif( option3 == "Surface" ):
                                contacts = 0

                                elements = [ min(g_opt[1: thetaDiv * yDiv + 1]), min(g_opt[ thetaDiv * yDiv + 1 : thetaDiv * yDiv * 2 + 1 ]),\
                                             min(g_opt[ thetaDiv * yDiv * 2 + 1 : thetaDiv * yDiv * 3 + 1 ]), min(g_opt[ thetaDiv * yDiv * 3 + 1 :]) ]

                                for element in elements:
                                    if( element <= self.wheelRadius * math.pow(10, -1) ):
                                        contacts += 1

                            #   Check if each wheel touches the ground
                            if( contacts >= 3 ):
                                x_opt = sol["x"].elements()

                                _roll_opt = x_opt[1]
                                _pitch_opt = x_opt[2]
                                
                                #   Retrieve inclination
                                inclination = self._inclination(_roll_opt, _pitch_opt)
                                terrainTraversabilityMap[id, jd] = inclination

                            else:
                                inclination = math.pi/2
                                terrainTraversabilityMap[id, jd] = inclination
                            
                            fullSol = { "f": sol["f"].elements(), "lam_x": sol["lam_x"].elements(), "x": sol["x"].elements(), "g": sol["g"].elements() }
                            stats = {"t_wall_total": solver.stats()["t_wall_total"], "iter_count": solver.stats()["iter_count"], "success": solver.stats()["success"]}
                            
                            local_dict[str(yaw)] = {"stats": stats, "solution": fullSol, 'inclination': inclination, 'contacts': contacts}

                            list_inclinations += [ inclination ]
                        
                        local_dict['gradient'] = gradMap( [i, j] ).elements()
                        
                        results_dict[ str(n_opt) ] = local_dict

                        max_inclination = max( list_inclinations )

                        #   Traversability is defined as the inclination (sign is not kept)
                        terrainTraversabilityMap[id, jd] = max_inclination

                    elif( option1 == "maxTerrainRoughness" ):
                        
                        list_roughness = []

                        cellStart = time.time()

                        for yaw in np.arange(-math.pi, math.pi, 0.5):

                            list_heights = []

                            #   Left back wheel contact point
                            wheel_lb_x, wheel_lb_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, -self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, i, j, 0.0)
                            lb = np.array( [wheel_lb_x, wheel_lb_y] )
                            ###

                            #   Left front wheel contact point
                            wheel_lf_x, wheel_lf_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, i, j, 0.0)
                            lf = np.array( [wheel_lf_x, wheel_lf_y] )
                            ###

                            #   Right back wheel contact point
                            wheel_rb_x, wheel_rb_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, -self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, i, j, 0.0)
                            rb = np.array( [wheel_rb_x, wheel_rb_y] )
                            ###

                            #   Right front wheel contact point
                            wheel_rf_x, wheel_rf_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, i, j, 0.0)
                            rf = np.array( [wheel_rf_x, wheel_rf_y] )

                            min_x = min( wheel_lb_x, wheel_lf_x, wheel_rb_x, wheel_rf_x )
                            max_x = max( wheel_lb_x, wheel_lf_x, wheel_rb_x, wheel_rf_x )

                            min_y = min( wheel_lb_y, wheel_lf_y, wheel_rb_y, wheel_rf_y )
                            max_y = max( wheel_lb_y, wheel_lf_y, wheel_rb_y, wheel_rf_y )

                            if( min_x < x[0] or max_x > x[-1] or max_y > y[-1] or min_y < y[0] ):
                                roughness = None
                            
                            else:
                                for pos_x in np.arange(min_x, max_x, 0.09):
                                    for pos_y in np.arange(min_y, max_y, 0.09):

                                        m = np.array( [pos_x, pos_y] )

                                        #   Check if point is inside rectangle
                                        if( 0 < np.dot(m - lb, lf - lb) and np.dot(m - lb, lf - lb) < np.dot(lf - lb, lf - lb) and\
                                            0 < np.dot(m - lb, rb - lb) and np.dot(m - lb, rb - lb) < np.dot(rb - lb, rb - lb) ):
                                            x_ind, y_ind = self._frame_meters2Pix(pos_x, pos_y)
                                            
                                            list_heights += [ self.metersHeightmap[x_ind, y_ind] ]

                                list_heights = np.array( list_heights )

                                roughness = np.std( list_heights ).tolist()
                            
                            list_roughness += [roughness]
                        
                        flag = False

                        for element in list_roughness:
                            if( element == None ):
                                flag = True        
                                break
                        
                        if( flag ):
                            max_roughness = None

                        else:
                            max_roughness = max( list_roughness )

                        terrainTraversabilityMap[id, jd] = max_roughness

                        cellEnd = time.time() - cellStart

                        #   Cell processing time
                        results_dict[ str(n_opt) ] = {"roughness": max_roughness, "gradient": gradMap( [i, j] ).elements(), "time": cellEnd}

                    n_opt += 1

                jd += 1
            id += 1

        end = time.time() - start

        results_dict['nb_iter'] = n_opt
        
        if( option1 == "maxTerrainRoughness" ):
            print("Fill None cells of MaxTerrainRoughness cost map.\n")            
            print("Get maximum value of maximum terrain roughness cost map.")
            maxCostMap = np.nanmax(terrainTraversabilityMap)

            id = 0

            for i in x:
                jd = 0

                for j in y:
                    
                    #   Replace cell with None by maximum roughness value
                    if( math.isnan( terrainTraversabilityMap[id, jd] ) ):
                        terrainTraversabilityMap[id, jd] = maxCostMap

                    jd += 1    
                id += 1

        _folder = self.mapFolderDir + str(self.mapFolder)

        if( option1 == "maxTerrainRoughness" ):
            costMapFile = _folder + "/" + option1 + ".npy"
            costMapFileText = _folder + "/" + option1 + ".dat"

            pubDir = self.exportPlot2PubDir + option1 + "+" + str(self.mapFolder) + ".png"
        
        else:
            costMapFile = _folder + "/" + option1 + "+" + option3 + ".npy"
            costMapFileText = _folder + "/" + option1 + "+" + option3 + ".dat"

            pubDir = self.exportPlot2PubDir + option1 + "+" + option3 + "+" + str(self.mapFolder) + ".png"

        print("Save cost map on .npy and .dat formats.")
        self._saveMap(costMapFile, costMapFileText, terrainTraversabilityMap)

        print("Export cost map on .pdf format for publication.")
        if(option1 == "HeightMin_maxInclination" or option1 == "HeightMin"):
            self._export2Publish(terrainTraversabilityMap, pubDir, "inclination [rad]")

        elif(option1 == "maxTerrainRoughness"):
            self._export2Publish(terrainTraversabilityMap, pubDir, "roughness")

        #print("Plot cost map.")
        #self._plotHeightmap(terrainTraversabilityMap)

        print("\n")
        
        return results_dict

    def _setInitialGuess(self, x, y, yaw, continuousHeightmap):

        """
            Set initial guess according to specs
        """
        
        #   Left back wheel contact point with respect to world frame
        wheel_lb_x, wheel_lb_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, -self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, 0)
        wheel_lb_z = float( continuousHeightmap( [wheel_lb_x, wheel_lb_y] ) )

        #   Left front wheel contact point with respect to world frame
        wheel_lf_x, wheel_lf_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, self.wheelLonSeparation/2.0, self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, 0)
        wheel_lf_z = float( continuousHeightmap( [wheel_lf_x, wheel_lf_y] ) )

        #   Right back wheel contact point with respect to world frame
        wheel_rb_x, wheel_rb_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, -self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, 0)
        wheel_rb_z = float( continuousHeightmap( [wheel_rb_x, wheel_rb_y] ) )

        #   Right front wheel position with respect to world frame
        wheel_rf_x, wheel_rf_y, _ = self._rotation3DBody2World(0.0, 0.0, yaw, self.wheelLonSeparation/2.0, -self.wheelLatSeparation/2.0, -self.wheelRadius, x, y, 0)
        wheel_rf_z = float( continuousHeightmap( [wheel_rf_x, wheel_rf_y] ) )
    
        contactPoints = np.array( [ [wheel_lb_x, wheel_lb_y, wheel_lb_z],\
                                    [wheel_lf_x, wheel_lf_y, wheel_lf_z],\
                                    [wheel_rb_x, wheel_rb_y, wheel_rb_z],\
                                    [wheel_rf_x, wheel_rf_y, wheel_rf_z] ] )

        #   Remove lowest height contact point
        ind = np.argsort( contactPoints[:, -1] )
        b = contactPoints[ind]

        b = np.delete(b, 0, axis = 0)

        #   Determine plane which contains highest three contact points
        vec1 = b[1, :] - b[0, :]
        vec2 = b[2, :] - b[0, :]

        normal2Plane = np.cross(vec2, vec1)
        
        #   Make sure normal to plane is pointing upwards
        if(normal2Plane[-1] < 0):
            normal2Plane = np.cross(vec1, vec2)
        
        #   Normalize plane normal vector
        normal2Plane = normal2Plane / np.linalg.norm( normal2Plane )
        
        roll = math.asin( -math.cos(yaw) * normal2Plane[1] + math.sin(yaw) * normal2Plane[0] )
        pitch = math.acos( normal2Plane[2] / math.cos(roll) )

        if( roll > math.pi / 4 ):
            #print("Roll outside bounds.")
            roll = math.pi / 4
        
        elif( roll < -math.pi / 4 ):
            #print("Roll outside bounds.")
            roll = -math.pi / 4

        if( pitch > math.pi / 4 ):
            #print("Pitch outside bounds.")
            pitch = math.pi / 4
        
        elif( pitch < -math.pi / 4 ):
            #print("Pitch outside bounds.")
            pitch = -math.pi / 4

        return [ float( continuousHeightmap( [x, y] ) ) + 0.5, roll, pitch]
    
    def _worstCaseScenarioMap(self, option3):
        
        """
            Compute worst-case scenario map among maxTerrainRoughness and HeightMin_maxInclination maps for each case
        """

        print("*** on function _worstCaseScenarioMap ***")

        x, y, cont_heightmap = self._continuousHeightmap_casadi()

        #   Generate elevation gradient for the sake of results discussion
        gradMap = self._heightmapGradient(cont_heightmap)

        inclination = self._loadCostMap("HeightMin_maxInclination", option3)
        roughness = self._loadCostMap("maxTerrainRoughness", option3)

        #   Normalize maps
        inclination = inclination / np.amax(inclination)
        roughness = roughness / np.amax(roughness)

        shapes = np.shape( inclination )

        worstCaseMap = np.zeros( ( shapes[0], shapes[1] ) )

        results_dict = {}

        n_opt = 0

        for i in range( shapes[0] ):
            for j in range( shapes[1] ):
                
                start = time.time()

                if( inclination[i, j] >= roughness[i, j] ):
                    worstCaseMap[i, j] = inclination[i, j]
                
                else:
                    worstCaseMap[i, j] = roughness[i, j]
                
                timeCell = time.time() - start

                i_mt, j_mt = self._frame_pix2Meters(i, j, worstCaseMap)
                
                results_dict[str(n_opt)] = {"cost": worstCaseMap[i, j], "time": timeCell, "gradient": gradMap( [i_mt, j_mt] ).elements()}
                
                n_opt += 1
        
        results_dict['nb_iter'] = n_opt
        
        _folder = self.mapFolderDir + str(self.mapFolder)
        
        costMapFile = _folder + "/" + "worstCase" + "+" + option3 + ".npy"
        costMapFileText = _folder + "/" + "worstCase" + "+" + option3 + ".dat"

        pubDir = self.exportPlot2PubDir + "worstCase" + "+" + option3 + "+" + str(self.mapFolder) + ".png"
        
        print("Save cost map on .npy and .dat formats.")
        self._saveMap(costMapFile, costMapFileText, worstCaseMap)

        print("Export cost map on .pdf format for publication.")
        self._export2Publish(worstCaseMap, pubDir, "cost")

        return worstCaseMap, results_dict
    
    def _refineCostMap(self, minObs, distance, option3, alpha, beta):

        """
            Fast marching 2: refine cost map around points below some threshold to decrease cost around that area

            :prevCostMap cost map obtained through standard method
            :minObs threshold
            :distance maximum radius around threshold where cost will decrease
        """
        
        print("*** on function _refineCostMap ***")

        costMap = self._loadCostMap("worstCase", option3)
        
        #newCostMap = np.zeros_like(prevCostMap)

        #   Convert meters to pixels
        x_pix = distance * 2 * costMap.shape[0] / self.mapDimx
        #y_pix = 2 * distance * self.metersHeightmap.shape[1] / self.mapDimy

        diameter = int(x_pix)

        #   Radius of circle centered on some position with a radius expressed on pixel units
        radius = int( diameter/2 )

        shape0 = costMap.shape[0]
        shape1 = costMap.shape[1]

        flag1 = True

        results_dict = {}

        #   Save points with cost above or equal to threshold
        for i in np.arange(0, shape0, 1):
            for j in np.arange(0, shape1, 1):

                #   save points with cost above threshold
                if( costMap[i, j] >= minObs ):

                    if(flag1):
                        listPoints = np.array( [i, j] )
                        flag1 = False
                    
                    else:
                        listPoints = np.vstack( ( listPoints, np.array( [i, j] ) ) )
                
                else:
                    pass
        
        shape_0 = listPoints.shape[0]
        shape_1 = listPoints.shape[1]

        listPoints2Change = {}

        for i in np.arange(0, shape_0, 1):
            
            point = listPoints[i, :]

            #   Get points inside circle around "point"
            min_i, max_i = point[0] - radius, point[0] + radius
            min_j, max_j = point[1] - radius, point[1] + radius

            for i2 in range(min_i, max_i + 1):
                for j2 in range(min_j, max_j + 1):
                    
                    dist = np.linalg.norm( np.array( [i2, j2] ) - point )

                    refinedValue = costMap[ point[0], point[1] ] * ( -math.pow(dist/radius, 2) + 1 )

                    if( i2 < 0 or i2 >= shape0 or j2 < 0 or j2 >= shape1 or dist > radius ):
                        pass
            
                    else:
                        strpoint = str( i2 ) + " " + str( j2 )

                        if( strpoint in listPoints2Change.keys() ):
                            listPoints2Change[strpoint] += [refinedValue]

                        else:
                            listPoints2Change[strpoint] = [ costMap[i2, j2], refinedValue]
        
        dictkeys  = listPoints2Change.keys()
        
        #   Replace respective points by maximum cost
        for key in dictkeys:
            maxCost = max( listPoints2Change[key] )
            
            stri3, strj3 = key.split(" ")
            i3 = int(stri3)
            j3 = int(strj3)
            
            costMap[i3, j3] = maxCost
        
        x, y, cont_heightmap = self._continuousHeightmap_casadi()

        #   Generate elevation gradient for the sake of results discussion
        gradMap = self._heightmapGradient(cont_heightmap)

        n_opt = 0

        for i in np.arange(0, shape0, 1):
            for j in np.arange(0, shape1, 1):
                i_mt, j_mt = self._frame_pix2Meters(i, j, costMap)

                results_dict[str(n_opt)] = {"gradient": gradMap( [i_mt, j_mt] ).elements(), "cost": costMap[i, j]}

                n_opt += 1
        
        results_dict["nb_iter"] = n_opt

        _folder = self.mapFolderDir + str(self.mapFolder)
        
        costMapFile = _folder + "/" + "mapRefinement" + "+" + option3 + "+" + str(alpha) + "+" + str(beta) + ".npy"
        costMapFileText = _folder + "/" + "mapRefinement" + "+" + option3 + "+" + str(alpha) + "+" + str(beta) + ".dat"

        pubDir = self.exportPlot2PubDir + "mapRefinement" + "+" + option3 + "+" + str(alpha) + "+" + str(beta) + "+" + str(self.mapFolder) + ".png"
        
        print("Save cost map on .npy and .dat formats.")
        self._saveMap(costMapFile, costMapFileText, costMap)

        print("Export cost map on .pdf format for publication.")
        self._export2Publish(costMap, pubDir, "cost")

        return costMap, results_dict

    def _inclination(self, roll, pitch):

        """
            Compute robot plane inclination from pitch and roll angles
        """

        return math.acos( math.cos( roll ) * math.cos( pitch ) )

    def _confidence_ellipse(self, x, y, ax, n_std = 3.0, facecolor = 'none', **kwargs):

        """
            Create a plot of the covariance confidence ellipse of *x* and *y*.

            Parameters
            ----------
            x, y : array-like, shape (n, )
                Input data.

            ax : matplotlib.axes.Axes
                The Axes object to draw the ellipse into.

            n_std : float
                The number of standard deviations to determine the ellipse's radiuses.

            **kwargs
                Forwarded to `~matplotlib.patches.Ellipse`

            Returns
            -------
            matplotlib.patches.Ellipse
        """

        if x.size != y.size:
            raise ValueError("x and y must be the same size")

        cov = np.cov(x, y)
        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensional dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                        facecolor=facecolor, **kwargs)

        # Calculating the standard deviation of x from
        # the squareroot of the variance and multiplying
        # with the given number of standard deviations.
        scale_x = np.sqrt(cov[0, 0]) * n_std
        mean_x = np.mean(x)

        # calculating the standard deviation of y ...
        scale_y = np.sqrt(cov[1, 1]) * n_std
        mean_y = np.mean(y)

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + ax.transData)
        
        return ax.add_patch(ellipse)

    def _limitsCircle(self, x, y):

        """
            Define circled area around which robot will turn

            :x cell index in dimension 0
            :y cell index in dimension 1
        """

        #   Compute diameter in meters
        robotCircleDiameter = math.sqrt( math.pow(self.wheelLonSeparation, 2) + math.pow(self.wheelLatSeparation, 2) )

        #   Radius
        radius = robotCircleDiameter / 2.0

        #   Define area limits in meters (virtual axis is the onde used here)
        top, bottom = x - radius, x + radius
        left, right = y - radius, y + radius

        return top, bottom, right, left
    
    def _isometricMap(self, costMap):
        
        """
            Get gradient potential throughout eikonal equation discretization
            
            :costMap cost map in numpy format
        """

        #X, Y, Z = self._getMapFrame(costMap, option = False)

        X, Y = self._getRealMeshgrid(costMap)

        phi = ( X - self.goalPoint[0] )**2 + ( Y - self.goalPoint[1] )**2 - 0.01

        isometricMap = skfmm.travel_time(phi, costMap, dx = ( self.mapDimx / costMap.shape[0], self.mapDimy / costMap.shape[1] ) )

        return isometricMap

    def _timeMatrixGradient(self, timeMap):

        # Apply Sobel operator
        #sobel_x = cv.Sobel(timeMap, cv.CV_64F, 1, 0, ksize=3)  # Gradient in X direction
        #sobel_y = cv.Sobel(timeMap, cv.CV_64F, 0, 1, ksize=3)  # Gradient in Y direction

        # Compute the gradient of the time matrix
        gradient_x, gradient_y = np.gradient(timeMap)

        return gradient_x, gradient_y
    
    def _continuousGradient_x(self, gx):

        x_mt, y_mt = self._getVirtualAxis(gx)

        data = gx.ravel(order = 'F')

        continuous_gx = ca.interpolant('continuous_gx', 'bspline', [x_mt, y_mt], data)

        return continuous_gx

    def _continuousGradient_y(self, gy):

        x_mt, y_mt = self._getVirtualAxis(gy)

        data = gy.ravel(order = 'F')

        continuous_gy = ca.interpolant('continuous_gy', 'bspline', [x_mt, y_mt], data)

        return continuous_gy

    def _potentialFlowGradient(self, timeMap):

        """
            Interpolate gradient potential in order to build a continuous R^2 -> R^2 gradient function
        """

        x_mt, y_mt = self._getVirtualAxis(timeMap)

        data = timeMap.ravel(order = 'F')

        print("Interpolate discrete map to yield a continuous heightmap")
        continuousTimeMap = ca.interpolant('continuousHeightmap', 'bspline', [x_mt, y_mt], data)

        x = ca.MX.sym('x', 2)

        timeMap_jac = ca.Function('map_jac', [x], [ ca.jacobian(continuousTimeMap(x), x) ] )

        return timeMap_jac

    def _getPath2Follow(self, timeMap, startingPoint, maxCycles = 500):

        shape0 = timeMap.shape[0]
        shape1 = timeMap.shape[1]

        i, j = self._transformation(startingPoint[0], startingPoint[1], option = "meters2PixRealFrame")

        if( i < 0 or i > shape0 - 1 or j < 0 or j > shape1 - 1 ):
            print("| Out of bounds position.")
            return None

        elif( isinstance( timeMap[ i, j ], np.ma.MaskedArray ) ):
            print("| Starting position is masked.")
            return None
        
        else:
            pass

        index = 0

        path = [startingPoint]

        last_time = timeMap[i, j]

        prev_x_mt = startingPoint[0]
        prev_y_mt = startingPoint[1]

        print("| Compute path by descent of time matrix.")
        while( index < maxCycles ):

            #print("(Last time, prev_x_mt, prev_y_mt): ", last_time, prev_x_mt, prev_y_mt)

            border_indexes = [ (i-1, j), (i-1, j-1), (i-1, j+1),\
                               (i, j-1), (i, j+1),\
                               (i+1, j), (i+1, j-1), (i+1, j+1) ]
        
            times = []
            timesMasked = []

            for element in border_indexes:
                
                #   Ignore masked cells
                if( isinstance( timeMap[ element[0], element[1] ], np.ma.MaskedArray ) ):
                    pass
                
                else:
                    times.append( timeMap[ element[0], element[1] ] )
                
                timesMasked.append( timeMap[ element[0], element[1] ] )

            if( all( isinstance(element, np.ma.MaskedArray) for element in times ) ):
                print("| Positions around current positions are all masked. Motion is impossible.")
                return None

            if( index > 0 ):
                last_time = next_time

            #   Determine lowest height centroid
            next_time = min(times)
            min_index = timesMasked.index(next_time)

            #print("Minimum index: ", min_index)

            i = border_indexes[min_index][0]
            j = border_indexes[min_index][1]

            #   Convert pixel coordinates to meters with respect to real frame
            x_mt, y_mt = self._transformation(i, j)

            #   Check if global minimum was achieved
            if( np.linalg.norm( np.array( [x_mt - self.goalPoint[0], y_mt - self.goalPoint[1]] ) ) <= self.goalCheck ):
                print("| Global minimum was achieved.\n")
                break

            elif( next_time >= last_time ):
                print("| Local minimum was achieved.\n")
                return None

            #   Add point to path
            if( np.linalg.norm( np.array( [x_mt - prev_x_mt, y_mt - prev_y_mt] ) ) >= self.pathGap ):
                prev_x_mt = x_mt
                prev_y_mt = y_mt
                path.append([x_mt, y_mt])
                index += 1
            
            else:
                pass

        #   Fill path with last position in case global minimum is achieved before
        while(index < maxCycles):
            path.append([x_mt, y_mt])

            index += 1

        return np.array(path)

    def _smoothPath(self, input):

        """
            Convert result from C function call in Python into a Float32MultiArray msg type

            :input  C instance of type SETTINGS
        """

        output_x = []
        output_y = []

        while input:
            output_x += [input.contents.x]
            output_y += [input.contents.y]

            input = input.contents.next  # Move to the next node
        
        # Fit a B-spline representation to the data
        tck, u = splprep( [ np.array(output_x), np.array(output_y) ], s = 0.1 )  # s is the smoothing factor

        # Generate new points along the smooth curve
        u_fine = np.linspace(0, 0.5, self.N + 1)  # Generate 100 smooth points
        x_smooth, y_smooth = splev(u_fine, tck)

        path2Follow_smooth = []

        for _ in range(self.N):
            yaw_smooth = math.atan2( y_smooth[_ + 1] - y_smooth[_], x_smooth[_ + 1] - x_smooth[_] )
            path2Follow_smooth += [x_smooth[_], y_smooth[_], yaw_smooth]
        
        path2Follow_smooth += [x_smooth[_ + 1], y_smooth[_ + 1], yaw_smooth]

        return path2Follow_smooth
        
    def _getPath2FollowIntegrator(self, timeMap, gradx, grady, startingPoint, maxCycles = 500, distance = 0):

        """
            Compute path by following gradient potential direction

            :ref numpy format matrix with row [x, y, 0, 0, 0, theta]
        """

        #   Set starting point
        px = startingPoint[0]
        py = startingPoint[1]

        prev_px = px
        prev_py = py

        index = 0

        flag = True

        if(distance != 0):
            pass
        
        else:
            distance = self.robotLength

        while( math.dist( self.goalPoint, [px, py] ) > self.goalCheck and index < maxCycles ):

            px_v, py_v = self._real2VirtualFrame(px, py)

            val_gradx = gradx( [px_v, py_v] ).elements()[0]
            val_grady = grady( [px_v, py_v] ).elements()[0]

            alpha = math.atan2( val_grady, val_gradx ) + math.pi                                      #   Compute heading at current point of the sequence
            grad_norm = math.sqrt( math.pow( val_gradx, 2 ) + math.pow( val_grady, 2 ) )                    #   Compute gradient modulus
            val_gradx = val_gradx / grad_norm
            val_grady = val_grady / grad_norm

            if( flag ):
                #   Add point with respective heading to the list
                ref_euler_v = np.array( [ px_v, py_v ] )
                ref_euler = np.array( [ px, py, 0, 0, 0, alpha] )
                flag = False
                index += 1

            #   Get path; Compute next sequence point (the modulus is replaced by self.pathDivision)
            k1_x = gradx( [px_v, py_v] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v, py_v] ).elements()[0], grady( [px_v, py_v] ).elements()[0] ] ) )
            k2_x = gradx( [px_v + self.pathDivision * k1_x / 2, py_v] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v + self.pathDivision * k1_x / 2, py_v] ).elements()[0], grady( [px_v + self.pathDivision * k1_x / 2, py_v] ).elements()[0] ] ) )
            k3_x = gradx( [px_v + self.pathDivision * k2_x / 2, py_v] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v + self.pathDivision * k2_x / 2, py_v] ).elements()[0], grady( [px_v + self.pathDivision * k2_x / 2, py_v] ).elements()[0] ] ) )
            k4_x = gradx( [px_v + self.pathDivision * k3_x, py_v] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v + self.pathDivision * k3_x, py_v] ).elements()[0], grady( [px_v + self.pathDivision * k3_x, py_v] ).elements()[0] ] ) ) 

            k1_y = grady( [px_v, py_v] ).elements()[0] / np.linalg.norm( np.array( [gradx( [px_v, py_v] ).elements()[0], grady( [px_v, py_v] ).elements()[0] ] ) )
            k2_y = grady( [px_v, py_v + self.pathDivision * k1_y / 2] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v, py_v + self.pathDivision * k1_y / 2] ).elements()[0], grady( [px_v, py_v + self.pathDivision * k1_y / 2] ).elements()[0] ] ) ) 
            k3_y = grady( [px_v, py_v + self.pathDivision * k2_y / 2] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v, py_v + self.pathDivision * k2_y / 2] ).elements()[0] , grady( [px_v, py_v + self.pathDivision * k2_y / 2] ).elements()[0] ] ) ) 
            k4_y = grady( [px_v, py_v + self.pathDivision * k3_y] ).elements()[0] / np.linalg.norm( np.array( [ gradx( [px_v, py_v + self.pathDivision * k3_y] ).elements()[0], grady( [px_v, py_v + self.pathDivision * k3_y] ).elements()[0] ] ) )

            px_v = px_v + self.pathDivision / 6 * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
            py_v = py_v + self.pathDivision / 6 * (k1_y + 2 * k2_y + 2 * k3_y + k4_y)

            px, py = self._virtual2RealFrame(px_v, py_v)

            next_ref_euler_v = np.array( [px_v, py_v] )
            next_ref_euler = np.array( [ px, py, 0, 0, 0, alpha] )

            ref_euler_v = np.vstack( (ref_euler_v, next_ref_euler_v) )
            ref_euler = np.vstack( ( ref_euler, next_ref_euler ) )

            """if( math.dist( [prev_px, prev_py], [px, py] ) >= distance ):
                alpha = math.atan2( grad_values[1], grad_values[0] )

                #   Add point with respective heading to the list
                next_ref_euler_v = np.array( [px_v, py_v] )
                next_ref_euler = np.array( [ px, py, 0, 0, 0, alpha] )

                ref_euler_v = np.vstack( (ref_euler_v, next_ref_euler_v) )
                ref_euler = np.vstack( ( ref_euler, next_ref_euler ) )

                prev_px = px
                prev_py = py

                index += 1"""
            
            index += 1
        
        alpha = math.atan2( val_grady, val_gradx )

        next_ref_euler = np.array( [ px, py, 0, 0, 0, alpha ] )

        #   Add points to path when goal point is near
        while( ref_euler.shape[0] < maxCycles ):
            ref_euler = np.vstack( ( ref_euler, next_ref_euler ) )

        return ref_euler, ref_euler_v

    def _getTrajectory2Follow(self, gradX, gradY, startingPoint, demonstration = False, maxCycles = 500):
        
        """
            Compute trajectory by following gradient potential direction

            :ref numpy format matrix with row [x, y, theta]
        """

        #   Set starting point
        px = startingPoint[0]
        py = startingPoint[1]

        index = 0

        if(demonstration is True):
            pass

        else:
            maxCycles = self.N + 1

        while( math.dist( self.goalPoint, [px, py] ) > self.goalCheck and index < maxCycles ):
            
            alpha = math.atan2( gradY(px, py)[0], gradX(px, py)[0] )                                                #   Compute heading at current point of the sequence
            mod = math.sqrt( math.pow( gradX(px, py)[0], 2 ) + math.pow( gradY(px, py)[0], 2 ) )                    #   Compute gradient modulus

            #print("Gradient modulus: ", mod)
            #print("Gradient modulus inverse: ", 1/mod)
            
            #   Add point with respective heading to the list
            if( index == 0 ):
                ref_euler = np.array( [ px, py, 0, 0, 0, alpha ] )              
                ref_quat = np.hstack( ( np.array( [ px, py, 0 ] ), self._euler2Quat( [0, 0, alpha] ) ) )
            
            elif( index > 0 ):
                next_ref_euler = np.array( [ px, py, 0, 0, 0, alpha ] )
                ref_euler = np.vstack( ( ref_euler, next_ref_euler ) )

                next_ref_quat = np.hstack( ( np.array( [ px, py, 0 ] ), self._euler2Quat( [0, 0, alpha] ) ) )
                ref_quat = np.vstack( ( ref_quat, next_ref_quat ) )

            #   Parameterize trajectory with respect to path
            px = px + 1 / mod * math.cos(alpha) * self.Ts
            py = py + 1 / mod * math.sin(alpha) * self.Ts
            
            index += 1

        #   Add points to path when goal point is near
        while( ref_euler.shape[0] < maxCycles and ref_quat.shape[0] < maxCycles ):
            ref_euler = np.vstack( ( ref_euler, next_ref_euler ) )
            ref_quat = np.vstack( ( ref_quat, next_ref_quat ) )

        if(demonstration is False):
            return list( ref_euler.ravel() ), list( ref_quat.ravel() )

        else:
            return ref_euler, ref_quat
    
    def _getSpeedMap(self, option1, option3, radius, threshold, maskValue, t1, a, b):

        """
            Get masked speed map and potential flow

            :option1            type of traversability map (worstCase   ;   mapRefinement)
            :option3            type of robot hull precision (Points  ;   Surface)
            :radius             parameter of map refinement
            :threshold          parameter of map refinement
            :maskValue          value assgined to masked cell on speedMap variable (equal to maskedSpeedMap but masked cells are assigned a value)
            :t1                 minimum untraversable cost
            :a                  parameter of of speed map computation function ( e^( ln(a) / b) * x ) )
            :b                  parameter of of speed map computation function ( e^( ln(a) / b) * x ) )
        """
        
        costMap = self._loadCostMap(option1, option3, radius = radius, threshold = threshold)

        #   Initialize map transformation parameters
        self._updateTransformationParameters(costMap)

        maskedSpeedMap = np.zeros( ( costMap.shape[0], costMap.shape[1] ) )
        speedMap = np.zeros( ( costMap.shape[0], costMap.shape[1] ) )

        for i in range( costMap.shape[0] ):
            for j in range( costMap.shape[1] ):
                if( costMap[i, j] >= t1 ):
                    speedMap[i, j] = maskValue
                    maskedSpeedMap[i, j] = 1

                else:
                    speedMap[i, j] = math.exp( math.log(a) / b * costMap[i, j])

        maskedSpeedMap = np.ma.masked_array(speedMap, mask = maskedSpeedMap)

        maskedPotentialFlow = self._isometricMap(maskedSpeedMap)

        return maskedSpeedMap, maskedPotentialFlow

    def _getGradientPotential(self, distance, minObs, fm2 = False):
        
        _folder = self.mapFolderDir + "/" + str(self.mapFolder)

        costMapFile = _folder + "/costmap.npy"
        
        #   Delete negative heights. Add minimum height to every height on map                     
        costMap = np.load(costMapFile)
        
        costMap = costMap / costMap.max() * self.maxSpeed

        if( fm2 ):
            fm2CostMap = self._fm2CostMap(costMap, minObs, distance)
            costMap = fm2CostMap

            costMap = costMap / costMap.max() * self.maxSpeed

        else:
            pass
        
        _X, _Y, _phi, _gradient_map, costMap = self._isometricMap(costMap)

        #_gradient_map = _gradient_map / _gradient_map.max()

        #self._plotHeightmap(_X, _Y, _gradient_map, threeD = False, titleName = "Potential with optimal path")

        return _gradient_map, _X, _Y