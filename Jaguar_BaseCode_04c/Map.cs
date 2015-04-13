using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;
        Navigation navigation = null;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;
        public double yOffset = 25;
        public double xOffset = 0;

        public int[] region1 = {0, 23};
        public int[] region2 = { 23, 217 };

        public Map(Navigation nav)
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:
            navigation = nav;

            numMapSegments = 217;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];


            #region original
            //top wall
            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
            mapSegmentCorners[0, 0, 1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;
            // left wall
            mapSegmentCorners[1, 0, 0] = -3.55 / 2;
            mapSegmentCorners[1, 0, 1] = 0.0;
            mapSegmentCorners[1, 1, 0] = -3.55 / 2;
            mapSegmentCorners[1, 1, 1] = -2.74;
            //right wall
            mapSegmentCorners[2, 0, 0] = 3.55 / 2;
            mapSegmentCorners[2, 0, 1] = 0.0;
            mapSegmentCorners[2, 1, 0] = 3.55 / 2;
            mapSegmentCorners[2, 1, 1] = -2.74;
            // upper right horizontal wall
            mapSegmentCorners[3, 0, 0] = 3.55 / 2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;
            // upper left horizontal wall
            mapSegmentCorners[4, 0, 0] = -3.55 / 2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55 / 2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            // lower left
            mapSegmentCorners[5, 0, 0] = -3.55 / 2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55 / 2 - 3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;
            // lower right
            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;
            // bottom wall
            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03 / 2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31; 
            #endregion

            #region extension of original
            // farthest left wall 
            mapSegmentCorners[12, 0, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[12, 0, 1] = 2.794;
            mapSegmentCorners[12, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[12, 1, 1] = -16;
            // farthest right wall
            mapSegmentCorners[13, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
            mapSegmentCorners[13, 0, 1] = 2.794;
            mapSegmentCorners[13, 1, 0] = 3.38 + 5.79 + 3.55 / 2;
            mapSegmentCorners[13, 1, 1] = -16;
            // far closer left wall
            mapSegmentCorners[14, 0, 0] = -3.55 / 2 - 5.79;
            mapSegmentCorners[14, 0, 1] = 0.0;
            mapSegmentCorners[14, 1, 0] = -3.55 / 2 - 5.79;
            mapSegmentCorners[14, 1, 1] = -13.23;
            // far closer right wall
            mapSegmentCorners[15, 0, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[15, 0, 1] = 0.0;
            mapSegmentCorners[15, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[15, 1, 1] = -13.23;
            // even closer left vertical
            mapSegmentCorners[8, 0, 0] = -3.55 / 2 - 3.05;
            mapSegmentCorners[8, 0, 1] = -2.74;
            mapSegmentCorners[8, 1, 0] = -3.55 / 2 - 3.05;
            mapSegmentCorners[8, 1, 1] = -13.23 + 2.74;
            // even closer right vertical
            mapSegmentCorners[9, 0, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[9, 0, 1] = -2.74;
            mapSegmentCorners[9, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[9, 1, 1] = -13.23 + 2.74;
            // bottom  left wall
            mapSegmentCorners[10, 0, 0] = -5.03 / 2;
            mapSegmentCorners[10, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[10, 1, 0] = -5.03 / 2;
            mapSegmentCorners[10, 1, 1] = -2.74 - 2.31 - 3.0; //added 3.0
            // bottom right wall
            mapSegmentCorners[11, 0, 0] = 5.03 / 2;
            mapSegmentCorners[11, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[11, 1, 0] = 5.03 / 2;
            mapSegmentCorners[11, 1, 1] = -2.74 - 2.31 - 3.0; //added 3.0 

            // lower right horizontal wall
            mapSegmentCorners[16, 0, 0] = 3.55 / 2;
            mapSegmentCorners[16, 0, 1] = -13.23;
            mapSegmentCorners[16, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[16, 1, 1] = -13.23;
            // lower left horizontal wall
            mapSegmentCorners[17, 0, 0] = -3.55 / 2;
            mapSegmentCorners[17, 0, 1] = -13.23;
            mapSegmentCorners[17, 1, 0] = -3.55 / 2 - 5.79;
            mapSegmentCorners[17, 1, 1] = -13.23;

            // symm left wall
            mapSegmentCorners[18, 0, 0] = -3.55 / 2;
            mapSegmentCorners[18, 0, 1] = -13.23;
            mapSegmentCorners[18, 1, 0] = -3.55 / 2;
            mapSegmentCorners[18, 1, 1] = -13.23 + 2.74;
            //symm right wall
            mapSegmentCorners[19, 0, 0] = 3.55 / 2;
            mapSegmentCorners[19, 0, 1] = -13.23;
            mapSegmentCorners[19, 1, 0] = 3.55 / 2;
            mapSegmentCorners[19, 1, 1] = -13.23 + 2.74;
            // symm lower left
            mapSegmentCorners[20, 0, 0] = -3.55 / 2;
            mapSegmentCorners[20, 0, 1] = -13.23 + 2.74;
            mapSegmentCorners[20, 1, 0] = -3.55 / 2 - 3.05;
            mapSegmentCorners[20, 1, 1] = -13.23 + 2.74;
            // symm lower right
            mapSegmentCorners[21, 0, 0] = 3.55 / 2;
            mapSegmentCorners[21, 0, 1] = -13.23 + 2.74;
            mapSegmentCorners[21, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[21, 1, 1] = -13.23 + 2.74;
            // bottom wall
            mapSegmentCorners[22, 0, 0] = 5.03 / 2;
            mapSegmentCorners[22, 0, 1] = -2.74 - 2.31 - 3.0; //not sure if these 3s are right
            mapSegmentCorners[22, 1, 0] = -5.03 / 2;
            mapSegmentCorners[22, 1, 1] = -2.74 - 2.31 - 3.0;
            #endregion
            

            #region columns
            mapSegmentCorners[23, 0, 0] = -10.920000; 
mapSegmentCorners[23, 0, 1] = -20.230000; 
mapSegmentCorners[23, 1, 0] = -10.620000; 
mapSegmentCorners[23, 1, 1] = -20.230000; 
mapSegmentCorners[24, 0, 0] = -10.920000; 
mapSegmentCorners[24, 0, 1] = -20.230000; 
mapSegmentCorners[24, 1, 0] = -10.920000; 
mapSegmentCorners[24, 1, 1] = -20.525000; 
mapSegmentCorners[25, 0, 0] = -10.920000; 
mapSegmentCorners[25, 0, 1] = -20.525000; 
mapSegmentCorners[25, 1, 0] = -10.620000; 
mapSegmentCorners[25, 1, 1] = -20.525000; 
mapSegmentCorners[26, 0, 0] = -10.620000; 
mapSegmentCorners[26, 0, 1] = -20.525000; 
mapSegmentCorners[26, 1, 0] = -10.620000; 
mapSegmentCorners[26, 1, 1] = -20.230000; 
mapSegmentCorners[27, 0, 0] = -10.920000; 
mapSegmentCorners[27, 0, 1] = -23.585000; 
mapSegmentCorners[27, 1, 0] = -10.620000; 
mapSegmentCorners[27, 1, 1] = -23.585000; 
mapSegmentCorners[28, 0, 0] = -10.920000; 
mapSegmentCorners[28, 0, 1] = -23.585000; 
mapSegmentCorners[28, 1, 0] = -10.920000; 
mapSegmentCorners[28, 1, 1] = -23.880000; 
mapSegmentCorners[29, 0, 0] = -10.920000; 
mapSegmentCorners[29, 0, 1] = -23.880000; 
mapSegmentCorners[29, 1, 0] = -10.620000; 
mapSegmentCorners[29, 1, 1] = -23.880000; 
mapSegmentCorners[30, 0, 0] = -10.620000; 
mapSegmentCorners[30, 0, 1] = -23.880000; 
mapSegmentCorners[30, 1, 0] = -10.620000; 
mapSegmentCorners[30, 1, 1] = -23.585000; 
mapSegmentCorners[31, 0, 0] = -7.270000; 
mapSegmentCorners[31, 0, 1] = -20.230000; 
mapSegmentCorners[31, 1, 0] = -6.970000; 
mapSegmentCorners[31, 1, 1] = -20.230000; 
mapSegmentCorners[32, 0, 0] = -7.270000; 
mapSegmentCorners[32, 0, 1] = -20.230000; 
mapSegmentCorners[32, 1, 0] = -7.270000; 
mapSegmentCorners[32, 1, 1] = -20.525000; 
mapSegmentCorners[33, 0, 0] = -7.270000; 
mapSegmentCorners[33, 0, 1] = -20.525000; 
mapSegmentCorners[33, 1, 0] = -6.970000; 
mapSegmentCorners[33, 1, 1] = -20.525000; 
mapSegmentCorners[34, 0, 0] = -6.970000; 
mapSegmentCorners[34, 0, 1] = -20.525000; 
mapSegmentCorners[34, 1, 0] = -6.970000; 
mapSegmentCorners[34, 1, 1] = -20.230000; 
mapSegmentCorners[35, 0, 0] = -7.270000; 
mapSegmentCorners[35, 0, 1] = -23.585000; 
mapSegmentCorners[35, 1, 0] = -6.970000; 
mapSegmentCorners[35, 1, 1] = -23.585000; 
mapSegmentCorners[36, 0, 0] = -7.270000; 
mapSegmentCorners[36, 0, 1] = -23.585000; 
mapSegmentCorners[36, 1, 0] = -7.270000; 
mapSegmentCorners[36, 1, 1] = -23.880000; 
mapSegmentCorners[37, 0, 0] = -7.270000; 
mapSegmentCorners[37, 0, 1] = -23.880000; 
mapSegmentCorners[37, 1, 0] = -6.970000; 
mapSegmentCorners[37, 1, 1] = -23.880000; 
mapSegmentCorners[38, 0, 0] = -6.970000; 
mapSegmentCorners[38, 0, 1] = -23.880000; 
mapSegmentCorners[38, 1, 0] = -6.970000; 
mapSegmentCorners[38, 1, 1] = -23.585000; 
mapSegmentCorners[39, 0, 0] = -3.620000; 
mapSegmentCorners[39, 0, 1] = -20.230000; 
mapSegmentCorners[39, 1, 0] = -3.320000; 
mapSegmentCorners[39, 1, 1] = -20.230000; 
mapSegmentCorners[40, 0, 0] = -3.620000; 
mapSegmentCorners[40, 0, 1] = -20.230000; 
mapSegmentCorners[40, 1, 0] = -3.620000; 
mapSegmentCorners[40, 1, 1] = -20.525000; 
mapSegmentCorners[41, 0, 0] = -3.620000; 
mapSegmentCorners[41, 0, 1] = -20.525000; 
mapSegmentCorners[41, 1, 0] = -3.320000; 
mapSegmentCorners[41, 1, 1] = -20.525000; 
mapSegmentCorners[42, 0, 0] = -3.320000; 
mapSegmentCorners[42, 0, 1] = -20.525000; 
mapSegmentCorners[42, 1, 0] = -3.320000; 
mapSegmentCorners[42, 1, 1] = -20.230000; 
mapSegmentCorners[43, 0, 0] = -3.620000; 
mapSegmentCorners[43, 0, 1] = -23.585000; 
mapSegmentCorners[43, 1, 0] = -3.320000; 
mapSegmentCorners[43, 1, 1] = -23.585000; 
mapSegmentCorners[44, 0, 0] = -3.620000; 
mapSegmentCorners[44, 0, 1] = -23.585000; 
mapSegmentCorners[44, 1, 0] = -3.620000; 
mapSegmentCorners[44, 1, 1] = -23.880000; 
mapSegmentCorners[45, 0, 0] = -3.620000; 
mapSegmentCorners[45, 0, 1] = -23.880000; 
mapSegmentCorners[45, 1, 0] = -3.320000; 
mapSegmentCorners[45, 1, 1] = -23.880000; 
mapSegmentCorners[46, 0, 0] = -3.320000; 
mapSegmentCorners[46, 0, 1] = -23.880000; 
mapSegmentCorners[46, 1, 0] = -3.320000; 
mapSegmentCorners[46, 1, 1] = -23.585000; 
mapSegmentCorners[47, 0, 0] = 0.030000; 
mapSegmentCorners[47, 0, 1] = -20.230000; 
mapSegmentCorners[47, 1, 0] = 0.330000; 
mapSegmentCorners[47, 1, 1] = -20.230000; 
mapSegmentCorners[48, 0, 0] = 0.030000; 
mapSegmentCorners[48, 0, 1] = -20.230000; 
mapSegmentCorners[48, 1, 0] = 0.030000; 
mapSegmentCorners[48, 1, 1] = -20.525000; 
mapSegmentCorners[49, 0, 0] = 0.030000; 
mapSegmentCorners[49, 0, 1] = -20.525000; 
mapSegmentCorners[49, 1, 0] = 0.330000; 
mapSegmentCorners[49, 1, 1] = -20.525000; 
mapSegmentCorners[50, 0, 0] = 0.330000; 
mapSegmentCorners[50, 0, 1] = -20.525000; 
mapSegmentCorners[50, 1, 0] = 0.330000; 
mapSegmentCorners[50, 1, 1] = -20.230000; 
mapSegmentCorners[51, 0, 0] = 0.030000; 
mapSegmentCorners[51, 0, 1] = -23.585000; 
mapSegmentCorners[51, 1, 0] = 0.330000; 
mapSegmentCorners[51, 1, 1] = -23.585000; 
mapSegmentCorners[52, 0, 0] = 0.030000; 
mapSegmentCorners[52, 0, 1] = -23.585000; 
mapSegmentCorners[52, 1, 0] = 0.030000; 
mapSegmentCorners[52, 1, 1] = -23.880000; 
mapSegmentCorners[53, 0, 0] = 0.030000; 
mapSegmentCorners[53, 0, 1] = -23.880000; 
mapSegmentCorners[53, 1, 0] = 0.330000; 
mapSegmentCorners[53, 1, 1] = -23.880000; 
mapSegmentCorners[54, 0, 0] = 0.330000; 
mapSegmentCorners[54, 0, 1] = -23.880000; 
mapSegmentCorners[54, 1, 0] = 0.330000; 
mapSegmentCorners[54, 1, 1] = -23.585000; 
mapSegmentCorners[55, 0, 0] = 3.680000; 
mapSegmentCorners[55, 0, 1] = -20.230000; 
mapSegmentCorners[55, 1, 0] = 3.980000; 
mapSegmentCorners[55, 1, 1] = -20.230000; 
mapSegmentCorners[56, 0, 0] = 3.680000; 
mapSegmentCorners[56, 0, 1] = -20.230000; 
mapSegmentCorners[56, 1, 0] = 3.680000; 
mapSegmentCorners[56, 1, 1] = -20.525000; 
mapSegmentCorners[57, 0, 0] = 3.680000; 
mapSegmentCorners[57, 0, 1] = -20.525000; 
mapSegmentCorners[57, 1, 0] = 3.980000; 
mapSegmentCorners[57, 1, 1] = -20.525000; 
mapSegmentCorners[58, 0, 0] = 3.980000; 
mapSegmentCorners[58, 0, 1] = -20.525000; 
mapSegmentCorners[58, 1, 0] = 3.980000; 
mapSegmentCorners[58, 1, 1] = -20.230000; 
mapSegmentCorners[59, 0, 0] = 3.680000; 
mapSegmentCorners[59, 0, 1] = -23.585000; 
mapSegmentCorners[59, 1, 0] = 3.980000; 
mapSegmentCorners[59, 1, 1] = -23.585000; 
mapSegmentCorners[60, 0, 0] = 3.680000; 
mapSegmentCorners[60, 0, 1] = -23.585000; 
mapSegmentCorners[60, 1, 0] = 3.680000; 
mapSegmentCorners[60, 1, 1] = -23.880000; 
mapSegmentCorners[61, 0, 0] = 3.680000; 
mapSegmentCorners[61, 0, 1] = -23.880000; 
mapSegmentCorners[61, 1, 0] = 3.980000; 
mapSegmentCorners[61, 1, 1] = -23.880000; 
mapSegmentCorners[62, 0, 0] = 3.980000; 
mapSegmentCorners[62, 0, 1] = -23.880000; 
mapSegmentCorners[62, 1, 0] = 3.980000; 
mapSegmentCorners[62, 1, 1] = -23.585000; 
mapSegmentCorners[63, 0, 0] = 7.330000; 
mapSegmentCorners[63, 0, 1] = -20.230000; 
mapSegmentCorners[63, 1, 0] = 7.630000; 
mapSegmentCorners[63, 1, 1] = -20.230000; 
mapSegmentCorners[64, 0, 0] = 7.330000; 
mapSegmentCorners[64, 0, 1] = -20.230000; 
mapSegmentCorners[64, 1, 0] = 7.330000; 
mapSegmentCorners[64, 1, 1] = -20.525000; 
mapSegmentCorners[65, 0, 0] = 7.330000; 
mapSegmentCorners[65, 0, 1] = -20.525000; 
mapSegmentCorners[65, 1, 0] = 7.630000; 
mapSegmentCorners[65, 1, 1] = -20.525000; 
mapSegmentCorners[66, 0, 0] = 7.630000; 
mapSegmentCorners[66, 0, 1] = -20.525000; 
mapSegmentCorners[66, 1, 0] = 7.630000; 
mapSegmentCorners[66, 1, 1] = -20.230000; 
mapSegmentCorners[67, 0, 0] = 7.330000; 
mapSegmentCorners[67, 0, 1] = -23.585000; 
mapSegmentCorners[67, 1, 0] = 7.630000; 
mapSegmentCorners[67, 1, 1] = -23.585000; 
mapSegmentCorners[68, 0, 0] = 7.330000; 
mapSegmentCorners[68, 0, 1] = -23.585000; 
mapSegmentCorners[68, 1, 0] = 7.330000; 
mapSegmentCorners[68, 1, 1] = -23.880000; 
mapSegmentCorners[69, 0, 0] = 7.330000; 
mapSegmentCorners[69, 0, 1] = -23.880000; 
mapSegmentCorners[69, 1, 0] = 7.630000; 
mapSegmentCorners[69, 1, 1] = -23.880000; 
mapSegmentCorners[70, 0, 0] = 7.630000; 
mapSegmentCorners[70, 0, 1] = -23.880000; 
mapSegmentCorners[70, 1, 0] = 7.630000; 
mapSegmentCorners[70, 1, 1] = -23.585000; 
mapSegmentCorners[71, 0, 0] = 10.980000; 
mapSegmentCorners[71, 0, 1] = -20.230000; 
mapSegmentCorners[71, 1, 0] = 11.280000; 
mapSegmentCorners[71, 1, 1] = -20.230000; 
mapSegmentCorners[72, 0, 0] = 10.980000; 
mapSegmentCorners[72, 0, 1] = -20.230000; 
mapSegmentCorners[72, 1, 0] = 10.980000; 
mapSegmentCorners[72, 1, 1] = -20.525000; 
mapSegmentCorners[73, 0, 0] = 10.980000; 
mapSegmentCorners[73, 0, 1] = -20.525000; 
mapSegmentCorners[73, 1, 0] = 11.280000; 
mapSegmentCorners[73, 1, 1] = -20.525000; 
mapSegmentCorners[74, 0, 0] = 11.280000; 
mapSegmentCorners[74, 0, 1] = -20.525000; 
mapSegmentCorners[74, 1, 0] = 11.280000; 
mapSegmentCorners[74, 1, 1] = -20.230000; 
mapSegmentCorners[75, 0, 0] = 10.980000; 
mapSegmentCorners[75, 0, 1] = -23.585000; 
mapSegmentCorners[75, 1, 0] = 11.280000; 
mapSegmentCorners[75, 1, 1] = -23.585000; 
mapSegmentCorners[76, 0, 0] = 10.980000; 
mapSegmentCorners[76, 0, 1] = -23.585000; 
mapSegmentCorners[76, 1, 0] = 10.980000; 
mapSegmentCorners[76, 1, 1] = -23.880000; 
mapSegmentCorners[77, 0, 0] = 10.980000; 
mapSegmentCorners[77, 0, 1] = -23.880000; 
mapSegmentCorners[77, 1, 0] = 11.280000; 
mapSegmentCorners[77, 1, 1] = -23.880000; 
mapSegmentCorners[78, 0, 0] = 11.280000; 
mapSegmentCorners[78, 0, 1] = -23.880000; 
mapSegmentCorners[78, 1, 0] = 11.280000; 
mapSegmentCorners[78, 1, 1] = -23.585000; 
mapSegmentCorners[79, 0, 0] = 14.630000; 
mapSegmentCorners[79, 0, 1] = -20.230000; 
mapSegmentCorners[79, 1, 0] = 14.930000; 
mapSegmentCorners[79, 1, 1] = -20.230000; 
mapSegmentCorners[80, 0, 0] = 14.630000; 
mapSegmentCorners[80, 0, 1] = -20.230000; 
mapSegmentCorners[80, 1, 0] = 14.630000; 
mapSegmentCorners[80, 1, 1] = -20.525000; 
mapSegmentCorners[81, 0, 0] = 14.630000; 
mapSegmentCorners[81, 0, 1] = -20.525000; 
mapSegmentCorners[81, 1, 0] = 14.930000; 
mapSegmentCorners[81, 1, 1] = -20.525000; 
mapSegmentCorners[82, 0, 0] = 14.930000; 
mapSegmentCorners[82, 0, 1] = -20.525000; 
mapSegmentCorners[82, 1, 0] = 14.930000; 
mapSegmentCorners[82, 1, 1] = -20.230000; 
mapSegmentCorners[83, 0, 0] = 14.630000; 
mapSegmentCorners[83, 0, 1] = -23.585000; 
mapSegmentCorners[83, 1, 0] = 14.930000; 
mapSegmentCorners[83, 1, 1] = -23.585000; 
mapSegmentCorners[84, 0, 0] = 14.630000; 
mapSegmentCorners[84, 0, 1] = -23.585000; 
mapSegmentCorners[84, 1, 0] = 14.630000; 
mapSegmentCorners[84, 1, 1] = -23.880000; 
mapSegmentCorners[85, 0, 0] = 14.630000; 
mapSegmentCorners[85, 0, 1] = -23.880000; 
mapSegmentCorners[85, 1, 0] = 14.930000; 
mapSegmentCorners[85, 1, 1] = -23.880000; 
mapSegmentCorners[86, 0, 0] = 14.930000; 
mapSegmentCorners[86, 0, 1] = -23.880000; 
mapSegmentCorners[86, 1, 0] = 14.930000; 
mapSegmentCorners[86, 1, 1] = -23.585000; 
mapSegmentCorners[87, 0, 0] = 18.280000; 
mapSegmentCorners[87, 0, 1] = -20.230000; 
mapSegmentCorners[87, 1, 0] = 18.580000; 
mapSegmentCorners[87, 1, 1] = -20.230000; 
mapSegmentCorners[88, 0, 0] = 18.280000; 
mapSegmentCorners[88, 0, 1] = -20.230000; 
mapSegmentCorners[88, 1, 0] = 18.280000; 
mapSegmentCorners[88, 1, 1] = -20.525000; 
mapSegmentCorners[89, 0, 0] = 18.280000; 
mapSegmentCorners[89, 0, 1] = -20.525000; 
mapSegmentCorners[89, 1, 0] = 18.580000; 
mapSegmentCorners[89, 1, 1] = -20.525000; 
mapSegmentCorners[90, 0, 0] = 18.580000; 
mapSegmentCorners[90, 0, 1] = -20.525000; 
mapSegmentCorners[90, 1, 0] = 18.580000; 
mapSegmentCorners[90, 1, 1] = -20.230000; 
mapSegmentCorners[91, 0, 0] = 18.280000; 
mapSegmentCorners[91, 0, 1] = -23.585000; 
mapSegmentCorners[91, 1, 0] = 18.580000; 
mapSegmentCorners[91, 1, 1] = -23.585000; 
mapSegmentCorners[92, 0, 0] = 18.280000; 
mapSegmentCorners[92, 0, 1] = -23.585000; 
mapSegmentCorners[92, 1, 0] = 18.280000; 
mapSegmentCorners[92, 1, 1] = -23.880000; 
mapSegmentCorners[93, 0, 0] = 18.280000; 
mapSegmentCorners[93, 0, 1] = -23.880000; 
mapSegmentCorners[93, 1, 0] = 18.580000; 
mapSegmentCorners[93, 1, 1] = -23.880000; 
mapSegmentCorners[94, 0, 0] = 18.580000; 
mapSegmentCorners[94, 0, 1] = -23.880000; 
mapSegmentCorners[94, 1, 0] = 18.580000; 
mapSegmentCorners[94, 1, 1] = -23.585000; 
mapSegmentCorners[95, 0, 0] = 18.280000; 
mapSegmentCorners[95, 0, 1] = -23.585000; 
mapSegmentCorners[95, 1, 0] = 18.575000; 
mapSegmentCorners[95, 1, 1] = -23.585000; 
mapSegmentCorners[96, 0, 0] = 18.280000; 
mapSegmentCorners[96, 0, 1] = -23.585000; 
mapSegmentCorners[96, 1, 0] = 18.280000; 
mapSegmentCorners[96, 1, 1] = -23.885000; 
mapSegmentCorners[97, 0, 0] = 18.280000; 
mapSegmentCorners[97, 0, 1] = -23.885000; 
mapSegmentCorners[97, 1, 0] = 18.575000; 
mapSegmentCorners[97, 1, 1] = -23.885000; 
mapSegmentCorners[98, 0, 0] = 18.575000; 
mapSegmentCorners[98, 0, 1] = -23.885000; 
mapSegmentCorners[98, 1, 0] = 18.575000; 
mapSegmentCorners[98, 1, 1] = -23.585000; 
mapSegmentCorners[99, 0, 0] = 18.280000; 
mapSegmentCorners[99, 0, 1] = -26.945000; 
mapSegmentCorners[99, 1, 0] = 18.575000; 
mapSegmentCorners[99, 1, 1] = -26.945000; 
mapSegmentCorners[100, 0, 0] = 18.280000; 
mapSegmentCorners[100, 0, 1] = -26.945000; 
mapSegmentCorners[100, 1, 0] = 18.280000; 
mapSegmentCorners[100, 1, 1] = -27.245000; 
mapSegmentCorners[101, 0, 0] = 18.280000; 
mapSegmentCorners[101, 0, 1] = -27.245000; 
mapSegmentCorners[101, 1, 0] = 18.575000; 
mapSegmentCorners[101, 1, 1] = -27.245000; 
mapSegmentCorners[102, 0, 0] = 18.575000; 
mapSegmentCorners[102, 0, 1] = -27.245000; 
mapSegmentCorners[102, 1, 0] = 18.575000; 
mapSegmentCorners[102, 1, 1] = -26.945000; 
mapSegmentCorners[103, 0, 0] = 18.280000; 
mapSegmentCorners[103, 0, 1] = -30.305000; 
mapSegmentCorners[103, 1, 0] = 18.575000; 
mapSegmentCorners[103, 1, 1] = -30.305000; 
mapSegmentCorners[104, 0, 0] = 18.280000; 
mapSegmentCorners[104, 0, 1] = -30.305000; 
mapSegmentCorners[104, 1, 0] = 18.280000; 
mapSegmentCorners[104, 1, 1] = -30.605000; 
mapSegmentCorners[105, 0, 0] = 18.280000; 
mapSegmentCorners[105, 0, 1] = -30.605000; 
mapSegmentCorners[105, 1, 0] = 18.575000; 
mapSegmentCorners[105, 1, 1] = -30.605000; 
mapSegmentCorners[106, 0, 0] = 18.575000; 
mapSegmentCorners[106, 0, 1] = -30.605000; 
mapSegmentCorners[106, 1, 0] = 18.575000; 
mapSegmentCorners[106, 1, 1] = -30.305000; 
mapSegmentCorners[107, 0, 0] = 18.280000; 
mapSegmentCorners[107, 0, 1] = -33.665000; 
mapSegmentCorners[107, 1, 0] = 18.575000; 
mapSegmentCorners[107, 1, 1] = -33.665000; 
mapSegmentCorners[108, 0, 0] = 18.280000; 
mapSegmentCorners[108, 0, 1] = -33.665000; 
mapSegmentCorners[108, 1, 0] = 18.280000; 
mapSegmentCorners[108, 1, 1] = -33.965000; 
mapSegmentCorners[109, 0, 0] = 18.280000; 
mapSegmentCorners[109, 0, 1] = -33.965000; 
mapSegmentCorners[109, 1, 0] = 18.575000; 
mapSegmentCorners[109, 1, 1] = -33.965000; 
mapSegmentCorners[110, 0, 0] = 18.575000; 
mapSegmentCorners[110, 0, 1] = -33.965000; 
mapSegmentCorners[110, 1, 0] = 18.575000; 
mapSegmentCorners[110, 1, 1] = -33.665000; 
mapSegmentCorners[111, 0, 0] = 21.925000; 
mapSegmentCorners[111, 0, 1] = -23.585000; 
mapSegmentCorners[111, 1, 0] = 21.925000; 
mapSegmentCorners[111, 1, 1] = -33.965000; 
mapSegmentCorners[112, 0, 0] = 18.280000; 
mapSegmentCorners[112, 0, 1] = -37.025000; 
mapSegmentCorners[112, 1, 0] = 18.575000; 
mapSegmentCorners[112, 1, 1] = -37.025000; 
mapSegmentCorners[113, 0, 0] = 18.280000; 
mapSegmentCorners[113, 0, 1] = -37.025000; 
mapSegmentCorners[113, 1, 0] = 18.280000; 
mapSegmentCorners[113, 1, 1] = -37.325000; 
mapSegmentCorners[114, 0, 0] = 18.280000; 
mapSegmentCorners[114, 0, 1] = -37.325000; 
mapSegmentCorners[114, 1, 0] = 18.575000; 
mapSegmentCorners[114, 1, 1] = -37.325000; 
mapSegmentCorners[115, 0, 0] = 18.575000; 
mapSegmentCorners[115, 0, 1] = -37.325000; 
mapSegmentCorners[115, 1, 0] = 18.575000; 
mapSegmentCorners[115, 1, 1] = -37.025000; 
mapSegmentCorners[116, 0, 0] = 21.925000; 
mapSegmentCorners[116, 0, 1] = -37.025000; 
mapSegmentCorners[116, 1, 0] = 22.220000; 
mapSegmentCorners[116, 1, 1] = -37.025000; 
mapSegmentCorners[117, 0, 0] = 21.925000; 
mapSegmentCorners[117, 0, 1] = -37.025000; 
mapSegmentCorners[117, 1, 0] = 21.925000; 
mapSegmentCorners[117, 1, 1] = -37.325000; 
mapSegmentCorners[118, 0, 0] = 21.925000; 
mapSegmentCorners[118, 0, 1] = -37.325000; 
mapSegmentCorners[118, 1, 0] = 22.220000; 
mapSegmentCorners[118, 1, 1] = -37.325000; 
mapSegmentCorners[119, 0, 0] = 22.220000; 
mapSegmentCorners[119, 0, 1] = -37.325000; 
mapSegmentCorners[119, 1, 0] = 22.220000; 
mapSegmentCorners[119, 1, 1] = -37.025000; 
mapSegmentCorners[120, 0, 0] = 18.280000; 
mapSegmentCorners[120, 0, 1] = -40.385000; 
mapSegmentCorners[120, 1, 0] = 18.575000; 
mapSegmentCorners[120, 1, 1] = -40.385000; 
mapSegmentCorners[121, 0, 0] = 18.280000; 
mapSegmentCorners[121, 0, 1] = -40.385000; 
mapSegmentCorners[121, 1, 0] = 18.280000; 
mapSegmentCorners[121, 1, 1] = -40.685000; 
mapSegmentCorners[122, 0, 0] = 18.280000; 
mapSegmentCorners[122, 0, 1] = -40.685000; 
mapSegmentCorners[122, 1, 0] = 18.575000; 
mapSegmentCorners[122, 1, 1] = -40.685000; 
mapSegmentCorners[123, 0, 0] = 18.575000; 
mapSegmentCorners[123, 0, 1] = -40.685000; 
mapSegmentCorners[123, 1, 0] = 18.575000; 
mapSegmentCorners[123, 1, 1] = -40.385000; 
mapSegmentCorners[124, 0, 0] = 21.925000; 
mapSegmentCorners[124, 0, 1] = -40.385000; 
mapSegmentCorners[124, 1, 0] = 22.220000; 
mapSegmentCorners[124, 1, 1] = -40.385000; 
mapSegmentCorners[125, 0, 0] = 21.925000; 
mapSegmentCorners[125, 0, 1] = -40.385000; 
mapSegmentCorners[125, 1, 0] = 21.925000; 
mapSegmentCorners[125, 1, 1] = -40.685000; 
mapSegmentCorners[126, 0, 0] = 21.925000; 
mapSegmentCorners[126, 0, 1] = -40.685000; 
mapSegmentCorners[126, 1, 0] = 22.220000; 
mapSegmentCorners[126, 1, 1] = -40.685000; 
mapSegmentCorners[127, 0, 0] = 22.220000; 
mapSegmentCorners[127, 0, 1] = -40.685000; 
mapSegmentCorners[127, 1, 0] = 22.220000; 
mapSegmentCorners[127, 1, 1] = -40.385000; 
mapSegmentCorners[128, 0, 0] = 18.280000; 
mapSegmentCorners[128, 0, 1] = -43.745000; 
mapSegmentCorners[128, 1, 0] = 18.575000; 
mapSegmentCorners[128, 1, 1] = -43.745000; 
mapSegmentCorners[129, 0, 0] = 18.280000; 
mapSegmentCorners[129, 0, 1] = -43.745000; 
mapSegmentCorners[129, 1, 0] = 18.280000; 
mapSegmentCorners[129, 1, 1] = -44.045000; 
mapSegmentCorners[130, 0, 0] = 18.280000; 
mapSegmentCorners[130, 0, 1] = -44.045000; 
mapSegmentCorners[130, 1, 0] = 18.575000; 
mapSegmentCorners[130, 1, 1] = -44.045000; 
mapSegmentCorners[131, 0, 0] = 18.575000; 
mapSegmentCorners[131, 0, 1] = -44.045000; 
mapSegmentCorners[131, 1, 0] = 18.575000; 
mapSegmentCorners[131, 1, 1] = -43.745000; 
mapSegmentCorners[132, 0, 0] = 18.280000; 
mapSegmentCorners[132, 0, 1] = -47.105000; 
mapSegmentCorners[132, 1, 0] = 18.575000; 
mapSegmentCorners[132, 1, 1] = -47.105000; 
mapSegmentCorners[133, 0, 0] = 18.280000; 
mapSegmentCorners[133, 0, 1] = -47.105000; 
mapSegmentCorners[133, 1, 0] = 18.280000; 
mapSegmentCorners[133, 1, 1] = -47.405000; 
mapSegmentCorners[134, 0, 0] = 18.280000; 
mapSegmentCorners[134, 0, 1] = -47.405000; 
mapSegmentCorners[134, 1, 0] = 18.575000; 
mapSegmentCorners[134, 1, 1] = -47.405000; 
mapSegmentCorners[135, 0, 0] = 18.575000; 
mapSegmentCorners[135, 0, 1] = -47.405000; 
mapSegmentCorners[135, 1, 0] = 18.575000; 
mapSegmentCorners[135, 1, 1] = -47.105000; 
mapSegmentCorners[136, 0, 0] = 18.280000; 
mapSegmentCorners[136, 0, 1] = -50.465000; 
mapSegmentCorners[136, 1, 0] = 18.575000; 
mapSegmentCorners[136, 1, 1] = -50.465000; 
mapSegmentCorners[137, 0, 0] = 18.280000; 
mapSegmentCorners[137, 0, 1] = -50.465000; 
mapSegmentCorners[137, 1, 0] = 18.280000; 
mapSegmentCorners[137, 1, 1] = -50.765000; 
mapSegmentCorners[138, 0, 0] = 18.280000; 
mapSegmentCorners[138, 0, 1] = -50.765000; 
mapSegmentCorners[138, 1, 0] = 18.575000; 
mapSegmentCorners[138, 1, 1] = -50.765000; 
mapSegmentCorners[139, 0, 0] = 18.575000; 
mapSegmentCorners[139, 0, 1] = -50.765000; 
mapSegmentCorners[139, 1, 0] = 18.575000; 
mapSegmentCorners[139, 1, 1] = -50.465000; 
mapSegmentCorners[140, 0, 0] = 18.280000; 
mapSegmentCorners[140, 0, 1] = -53.825000; 
mapSegmentCorners[140, 1, 0] = 18.575000; 
mapSegmentCorners[140, 1, 1] = -53.825000; 
mapSegmentCorners[141, 0, 0] = 18.280000; 
mapSegmentCorners[141, 0, 1] = -53.825000; 
mapSegmentCorners[141, 1, 0] = 18.280000; 
mapSegmentCorners[141, 1, 1] = -54.125000; 
mapSegmentCorners[142, 0, 0] = 18.280000; 
mapSegmentCorners[142, 0, 1] = -54.125000; 
mapSegmentCorners[142, 1, 0] = 18.575000; 
mapSegmentCorners[142, 1, 1] = -54.125000; 
mapSegmentCorners[143, 0, 0] = 18.575000; 
mapSegmentCorners[143, 0, 1] = -54.125000; 
mapSegmentCorners[143, 1, 0] = 18.575000; 
mapSegmentCorners[143, 1, 1] = -53.825000; 
mapSegmentCorners[144, 0, 0] = 21.925000; 
mapSegmentCorners[144, 0, 1] = -43.745000; 
mapSegmentCorners[144, 1, 0] = 21.925000; 
mapSegmentCorners[144, 1, 1] = -54.125000; 
mapSegmentCorners[145, 0, 0] = 18.280000; 
mapSegmentCorners[145, 0, 1] = -57.185000; 
mapSegmentCorners[145, 1, 0] = 18.580000; 
mapSegmentCorners[145, 1, 1] = -57.185000; 
mapSegmentCorners[146, 0, 0] = 18.280000; 
mapSegmentCorners[146, 0, 1] = -57.185000; 
mapSegmentCorners[146, 1, 0] = 18.280000; 
mapSegmentCorners[146, 1, 1] = -57.480000; 
mapSegmentCorners[147, 0, 0] = 18.280000; 
mapSegmentCorners[147, 0, 1] = -57.480000; 
mapSegmentCorners[147, 1, 0] = 18.580000; 
mapSegmentCorners[147, 1, 1] = -57.480000; 
mapSegmentCorners[148, 0, 0] = 18.580000; 
mapSegmentCorners[148, 0, 1] = -57.480000; 
mapSegmentCorners[148, 1, 0] = 18.580000; 
mapSegmentCorners[148, 1, 1] = -57.185000; 
mapSegmentCorners[149, 0, 0] = 18.280000; 
mapSegmentCorners[149, 0, 1] = -60.540000; 
mapSegmentCorners[149, 1, 0] = 18.580000; 
mapSegmentCorners[149, 1, 1] = -60.540000; 
mapSegmentCorners[150, 0, 0] = 18.280000; 
mapSegmentCorners[150, 0, 1] = -60.540000; 
mapSegmentCorners[150, 1, 0] = 18.280000; 
mapSegmentCorners[150, 1, 1] = -60.835000; 
mapSegmentCorners[151, 0, 0] = 18.280000; 
mapSegmentCorners[151, 0, 1] = -60.835000; 
mapSegmentCorners[151, 1, 0] = 18.580000; 
mapSegmentCorners[151, 1, 1] = -60.835000; 
mapSegmentCorners[152, 0, 0] = 18.580000; 
mapSegmentCorners[152, 0, 1] = -60.835000; 
mapSegmentCorners[152, 1, 0] = 18.580000; 
mapSegmentCorners[152, 1, 1] = -60.540000; 
mapSegmentCorners[153, 0, 0] = 14.630000; 
mapSegmentCorners[153, 0, 1] = -57.185000; 
mapSegmentCorners[153, 1, 0] = 14.930000; 
mapSegmentCorners[153, 1, 1] = -57.185000; 
mapSegmentCorners[154, 0, 0] = 14.630000; 
mapSegmentCorners[154, 0, 1] = -57.185000; 
mapSegmentCorners[154, 1, 0] = 14.630000; 
mapSegmentCorners[154, 1, 1] = -57.480000; 
mapSegmentCorners[155, 0, 0] = 14.630000; 
mapSegmentCorners[155, 0, 1] = -57.480000; 
mapSegmentCorners[155, 1, 0] = 14.930000; 
mapSegmentCorners[155, 1, 1] = -57.480000; 
mapSegmentCorners[156, 0, 0] = 14.930000; 
mapSegmentCorners[156, 0, 1] = -57.480000; 
mapSegmentCorners[156, 1, 0] = 14.930000; 
mapSegmentCorners[156, 1, 1] = -57.185000; 
mapSegmentCorners[157, 0, 0] = 14.630000; 
mapSegmentCorners[157, 0, 1] = -60.540000; 
mapSegmentCorners[157, 1, 0] = 14.930000; 
mapSegmentCorners[157, 1, 1] = -60.540000; 
mapSegmentCorners[158, 0, 0] = 14.630000; 
mapSegmentCorners[158, 0, 1] = -60.540000; 
mapSegmentCorners[158, 1, 0] = 14.630000; 
mapSegmentCorners[158, 1, 1] = -60.835000; 
mapSegmentCorners[159, 0, 0] = 14.630000; 
mapSegmentCorners[159, 0, 1] = -60.835000; 
mapSegmentCorners[159, 1, 0] = 14.930000; 
mapSegmentCorners[159, 1, 1] = -60.835000; 
mapSegmentCorners[160, 0, 0] = 14.930000; 
mapSegmentCorners[160, 0, 1] = -60.835000; 
mapSegmentCorners[160, 1, 0] = 14.930000; 
mapSegmentCorners[160, 1, 1] = -60.540000; 
mapSegmentCorners[161, 0, 0] = 10.980000; 
mapSegmentCorners[161, 0, 1] = -57.185000; 
mapSegmentCorners[161, 1, 0] = 11.280000; 
mapSegmentCorners[161, 1, 1] = -57.185000; 
mapSegmentCorners[162, 0, 0] = 10.980000; 
mapSegmentCorners[162, 0, 1] = -57.185000; 
mapSegmentCorners[162, 1, 0] = 10.980000; 
mapSegmentCorners[162, 1, 1] = -57.480000; 
mapSegmentCorners[163, 0, 0] = 10.980000; 
mapSegmentCorners[163, 0, 1] = -57.480000; 
mapSegmentCorners[163, 1, 0] = 11.280000; 
mapSegmentCorners[163, 1, 1] = -57.480000; 
mapSegmentCorners[164, 0, 0] = 11.280000; 
mapSegmentCorners[164, 0, 1] = -57.480000; 
mapSegmentCorners[164, 1, 0] = 11.280000; 
mapSegmentCorners[164, 1, 1] = -57.185000; 
mapSegmentCorners[165, 0, 0] = 10.980000; 
mapSegmentCorners[165, 0, 1] = -60.540000; 
mapSegmentCorners[165, 1, 0] = 11.280000; 
mapSegmentCorners[165, 1, 1] = -60.540000; 
mapSegmentCorners[166, 0, 0] = 10.980000; 
mapSegmentCorners[166, 0, 1] = -60.540000; 
mapSegmentCorners[166, 1, 0] = 10.980000; 
mapSegmentCorners[166, 1, 1] = -60.835000; 
mapSegmentCorners[167, 0, 0] = 10.980000; 
mapSegmentCorners[167, 0, 1] = -60.835000; 
mapSegmentCorners[167, 1, 0] = 11.280000; 
mapSegmentCorners[167, 1, 1] = -60.835000; 
mapSegmentCorners[168, 0, 0] = 11.280000; 
mapSegmentCorners[168, 0, 1] = -60.835000; 
mapSegmentCorners[168, 1, 0] = 11.280000; 
mapSegmentCorners[168, 1, 1] = -60.540000; 
mapSegmentCorners[169, 0, 0] = 7.330000; 
mapSegmentCorners[169, 0, 1] = -57.185000; 
mapSegmentCorners[169, 1, 0] = 7.630000; 
mapSegmentCorners[169, 1, 1] = -57.185000; 
mapSegmentCorners[170, 0, 0] = 7.330000; 
mapSegmentCorners[170, 0, 1] = -57.185000; 
mapSegmentCorners[170, 1, 0] = 7.330000; 
mapSegmentCorners[170, 1, 1] = -57.480000; 
mapSegmentCorners[171, 0, 0] = 7.330000; 
mapSegmentCorners[171, 0, 1] = -57.480000; 
mapSegmentCorners[171, 1, 0] = 7.630000; 
mapSegmentCorners[171, 1, 1] = -57.480000; 
mapSegmentCorners[172, 0, 0] = 7.630000; 
mapSegmentCorners[172, 0, 1] = -57.480000; 
mapSegmentCorners[172, 1, 0] = 7.630000; 
mapSegmentCorners[172, 1, 1] = -57.185000; 
mapSegmentCorners[173, 0, 0] = 7.330000; 
mapSegmentCorners[173, 0, 1] = -60.540000; 
mapSegmentCorners[173, 1, 0] = 7.630000; 
mapSegmentCorners[173, 1, 1] = -60.540000; 
mapSegmentCorners[174, 0, 0] = 7.330000; 
mapSegmentCorners[174, 0, 1] = -60.540000; 
mapSegmentCorners[174, 1, 0] = 7.330000; 
mapSegmentCorners[174, 1, 1] = -60.835000; 
mapSegmentCorners[175, 0, 0] = 7.330000; 
mapSegmentCorners[175, 0, 1] = -60.835000; 
mapSegmentCorners[175, 1, 0] = 7.630000; 
mapSegmentCorners[175, 1, 1] = -60.835000; 
mapSegmentCorners[176, 0, 0] = 7.630000; 
mapSegmentCorners[176, 0, 1] = -60.835000; 
mapSegmentCorners[176, 1, 0] = 7.630000; 
mapSegmentCorners[176, 1, 1] = -60.540000; 
mapSegmentCorners[177, 0, 0] = 3.680000; 
mapSegmentCorners[177, 0, 1] = -57.185000; 
mapSegmentCorners[177, 1, 0] = 3.980000; 
mapSegmentCorners[177, 1, 1] = -57.185000; 
mapSegmentCorners[178, 0, 0] = 3.680000; 
mapSegmentCorners[178, 0, 1] = -57.185000; 
mapSegmentCorners[178, 1, 0] = 3.680000; 
mapSegmentCorners[178, 1, 1] = -57.480000; 
mapSegmentCorners[179, 0, 0] = 3.680000; 
mapSegmentCorners[179, 0, 1] = -57.480000; 
mapSegmentCorners[179, 1, 0] = 3.980000; 
mapSegmentCorners[179, 1, 1] = -57.480000; 
mapSegmentCorners[180, 0, 0] = 3.980000; 
mapSegmentCorners[180, 0, 1] = -57.480000; 
mapSegmentCorners[180, 1, 0] = 3.980000; 
mapSegmentCorners[180, 1, 1] = -57.185000; 
mapSegmentCorners[181, 0, 0] = 3.680000; 
mapSegmentCorners[181, 0, 1] = -60.540000; 
mapSegmentCorners[181, 1, 0] = 3.980000; 
mapSegmentCorners[181, 1, 1] = -60.540000; 
mapSegmentCorners[182, 0, 0] = 3.680000; 
mapSegmentCorners[182, 0, 1] = -60.540000; 
mapSegmentCorners[182, 1, 0] = 3.680000; 
mapSegmentCorners[182, 1, 1] = -60.835000; 
mapSegmentCorners[183, 0, 0] = 3.680000; 
mapSegmentCorners[183, 0, 1] = -60.835000; 
mapSegmentCorners[183, 1, 0] = 3.980000; 
mapSegmentCorners[183, 1, 1] = -60.835000; 
mapSegmentCorners[184, 0, 0] = 3.980000; 
mapSegmentCorners[184, 0, 1] = -60.835000; 
mapSegmentCorners[184, 1, 0] = 3.980000; 
mapSegmentCorners[184, 1, 1] = -60.540000; 
mapSegmentCorners[185, 0, 0] = 0.030000; 
mapSegmentCorners[185, 0, 1] = -57.185000; 
mapSegmentCorners[185, 1, 0] = 0.330000; 
mapSegmentCorners[185, 1, 1] = -57.185000; 
mapSegmentCorners[186, 0, 0] = 0.030000; 
mapSegmentCorners[186, 0, 1] = -57.185000; 
mapSegmentCorners[186, 1, 0] = 0.030000; 
mapSegmentCorners[186, 1, 1] = -57.480000; 
mapSegmentCorners[187, 0, 0] = 0.030000; 
mapSegmentCorners[187, 0, 1] = -57.480000; 
mapSegmentCorners[187, 1, 0] = 0.330000; 
mapSegmentCorners[187, 1, 1] = -57.480000; 
mapSegmentCorners[188, 0, 0] = 0.330000; 
mapSegmentCorners[188, 0, 1] = -57.480000; 
mapSegmentCorners[188, 1, 0] = 0.330000; 
mapSegmentCorners[188, 1, 1] = -57.185000; 
mapSegmentCorners[189, 0, 0] = 0.030000; 
mapSegmentCorners[189, 0, 1] = -60.540000; 
mapSegmentCorners[189, 1, 0] = 0.330000; 
mapSegmentCorners[189, 1, 1] = -60.540000; 
mapSegmentCorners[190, 0, 0] = 0.030000; 
mapSegmentCorners[190, 0, 1] = -60.540000; 
mapSegmentCorners[190, 1, 0] = 0.030000; 
mapSegmentCorners[190, 1, 1] = -60.835000; 
mapSegmentCorners[191, 0, 0] = 0.030000; 
mapSegmentCorners[191, 0, 1] = -60.835000; 
mapSegmentCorners[191, 1, 0] = 0.330000; 
mapSegmentCorners[191, 1, 1] = -60.835000; 
mapSegmentCorners[192, 0, 0] = 0.330000; 
mapSegmentCorners[192, 0, 1] = -60.835000; 
mapSegmentCorners[192, 1, 0] = 0.330000; 
mapSegmentCorners[192, 1, 1] = -60.540000; 
mapSegmentCorners[193, 0, 0] = -3.620000; 
mapSegmentCorners[193, 0, 1] = -57.185000; 
mapSegmentCorners[193, 1, 0] = -3.320000; 
mapSegmentCorners[193, 1, 1] = -57.185000; 
mapSegmentCorners[194, 0, 0] = -3.620000; 
mapSegmentCorners[194, 0, 1] = -57.185000; 
mapSegmentCorners[194, 1, 0] = -3.620000; 
mapSegmentCorners[194, 1, 1] = -57.480000; 
mapSegmentCorners[195, 0, 0] = -3.620000; 
mapSegmentCorners[195, 0, 1] = -57.480000; 
mapSegmentCorners[195, 1, 0] = -3.320000; 
mapSegmentCorners[195, 1, 1] = -57.480000; 
mapSegmentCorners[196, 0, 0] = -3.320000; 
mapSegmentCorners[196, 0, 1] = -57.480000; 
mapSegmentCorners[196, 1, 0] = -3.320000; 
mapSegmentCorners[196, 1, 1] = -57.185000; 
mapSegmentCorners[197, 0, 0] = -3.620000; 
mapSegmentCorners[197, 0, 1] = -60.540000; 
mapSegmentCorners[197, 1, 0] = -3.320000; 
mapSegmentCorners[197, 1, 1] = -60.540000; 
mapSegmentCorners[198, 0, 0] = -3.620000; 
mapSegmentCorners[198, 0, 1] = -60.540000; 
mapSegmentCorners[198, 1, 0] = -3.620000; 
mapSegmentCorners[198, 1, 1] = -60.835000; 
mapSegmentCorners[199, 0, 0] = -3.620000; 
mapSegmentCorners[199, 0, 1] = -60.835000; 
mapSegmentCorners[199, 1, 0] = -3.320000; 
mapSegmentCorners[199, 1, 1] = -60.835000; 
mapSegmentCorners[200, 0, 0] = -3.320000; 
mapSegmentCorners[200, 0, 1] = -60.835000; 
mapSegmentCorners[200, 1, 0] = -3.320000; 
mapSegmentCorners[200, 1, 1] = -60.540000; 
mapSegmentCorners[201, 0, 0] = -7.270000; 
mapSegmentCorners[201, 0, 1] = -57.185000; 
mapSegmentCorners[201, 1, 0] = -6.970000; 
mapSegmentCorners[201, 1, 1] = -57.185000; 
mapSegmentCorners[202, 0, 0] = -7.270000; 
mapSegmentCorners[202, 0, 1] = -57.185000; 
mapSegmentCorners[202, 1, 0] = -7.270000; 
mapSegmentCorners[202, 1, 1] = -57.480000; 
mapSegmentCorners[203, 0, 0] = -7.270000; 
mapSegmentCorners[203, 0, 1] = -57.480000; 
mapSegmentCorners[203, 1, 0] = -6.970000; 
mapSegmentCorners[203, 1, 1] = -57.480000; 
mapSegmentCorners[204, 0, 0] = -6.970000; 
mapSegmentCorners[204, 0, 1] = -57.480000; 
mapSegmentCorners[204, 1, 0] = -6.970000; 
mapSegmentCorners[204, 1, 1] = -57.185000; 
mapSegmentCorners[205, 0, 0] = -7.270000; 
mapSegmentCorners[205, 0, 1] = -60.540000; 
mapSegmentCorners[205, 1, 0] = -6.970000; 
mapSegmentCorners[205, 1, 1] = -60.540000; 
mapSegmentCorners[206, 0, 0] = -7.270000; 
mapSegmentCorners[206, 0, 1] = -60.540000; 
mapSegmentCorners[206, 1, 0] = -7.270000; 
mapSegmentCorners[206, 1, 1] = -60.835000; 
mapSegmentCorners[207, 0, 0] = -7.270000; 
mapSegmentCorners[207, 0, 1] = -60.835000; 
mapSegmentCorners[207, 1, 0] = -6.970000; 
mapSegmentCorners[207, 1, 1] = -60.835000; 
mapSegmentCorners[208, 0, 0] = -6.970000; 
mapSegmentCorners[208, 0, 1] = -60.835000; 
mapSegmentCorners[208, 1, 0] = -6.970000; 
mapSegmentCorners[208, 1, 1] = -60.540000; 
mapSegmentCorners[209, 0, 0] = -10.920000; 
mapSegmentCorners[209, 0, 1] = -57.185000; 
mapSegmentCorners[209, 1, 0] = -10.620000; 
mapSegmentCorners[209, 1, 1] = -57.185000; 
mapSegmentCorners[210, 0, 0] = -10.920000; 
mapSegmentCorners[210, 0, 1] = -57.185000; 
mapSegmentCorners[210, 1, 0] = -10.920000; 
mapSegmentCorners[210, 1, 1] = -57.480000; 
mapSegmentCorners[211, 0, 0] = -10.920000; 
mapSegmentCorners[211, 0, 1] = -57.480000; 
mapSegmentCorners[211, 1, 0] = -10.620000; 
mapSegmentCorners[211, 1, 1] = -57.480000; 
mapSegmentCorners[212, 0, 0] = -10.620000; 
mapSegmentCorners[212, 0, 1] = -57.480000; 
mapSegmentCorners[212, 1, 0] = -10.620000; 
mapSegmentCorners[212, 1, 1] = -57.185000; 
mapSegmentCorners[213, 0, 0] = -10.920000; 
mapSegmentCorners[213, 0, 1] = -60.540000; 
mapSegmentCorners[213, 1, 0] = -10.620000; 
mapSegmentCorners[213, 1, 1] = -60.540000; 
mapSegmentCorners[214, 0, 0] = -10.920000; 
mapSegmentCorners[214, 0, 1] = -60.540000; 
mapSegmentCorners[214, 1, 0] = -10.920000; 
mapSegmentCorners[214, 1, 1] = -60.835000; 
mapSegmentCorners[215, 0, 0] = -10.920000; 
mapSegmentCorners[215, 0, 1] = -60.835000; 
mapSegmentCorners[215, 1, 0] = -10.620000; 
mapSegmentCorners[215, 1, 1] = -60.835000; 
mapSegmentCorners[216, 0, 0] = -10.620000; 
mapSegmentCorners[216, 0, 1] = -60.835000; 
mapSegmentCorners[216, 1, 0] = -10.620000; 
mapSegmentCorners[216, 1, 1] = -60.540000; 

            #endregion
            
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
                mapSegmentCorners[i, 0, 0] += xOffset;
                mapSegmentCorners[i, 1, 0] += xOffset;
                mapSegmentCorners[i, 0, 1] += yOffset;
                mapSegmentCorners[i, 1, 1] += yOffset;

		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
		
		        // Set wall segments to be horizontal
		        slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
		        intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){
            double testT = t;
            if (testT > Math.PI)
            {
                testT -= 2 * Math.PI;
            }
            if (testT < -Math.PI)
            {
                testT += 2 * Math.PI;
            }

            double wallX1 = mapSegmentCorners[segment, 0, 0];
            double wallY1 = mapSegmentCorners[segment, 0, 1];
            double wallX2 = mapSegmentCorners[segment, 1, 0];
            double wallY2 = mapSegmentCorners[segment, 1, 1];

            double pointSlope = Math.Tan(t);
            double pointIntercept = y - pointSlope * x;

            // calculation taken from community.topcoder.com/tc?module=Static&d1=tutorials&d2=geometry2
            double delta = (-pointSlope * 1) - (-slopes[segment]) * 1;
            if (delta == 0)
            {
                // lines are parallel, there is no intersection
                return double.PositiveInfinity;
            }
            double xIntersect = ((1 * pointIntercept) - (1 * intercepts[segment])) / delta;
            double yIntersect = ((-pointSlope * intercepts[segment]) - (-slopes[segment] * pointIntercept)) / delta;

            double epsilon = 0.01;

            // quadrant laser range is pointing in
            Boolean yUp = (testT > -epsilon);
            Boolean yDown = (testT < epsilon);
            Boolean xRight = ((Math.Abs(testT) < (Math.PI / 2 + epsilon)));
            Boolean xLeft = ((Math.Abs(testT) > (Math.PI / 2 - epsilon)));
            // quadrant of the intersection
            Boolean Quad1 = (yIntersect >= y && xIntersect >= x);
            Boolean Quad2 =  (yIntersect >= y && xIntersect < x);
            Boolean Quad3 =  (yIntersect <= y && xIntersect < x);
            Boolean Quad4 =  (yIntersect < y && xIntersect >= x);

            // checking if intersection is actually in laser sight rather than behind
            if (!((Quad1 && yUp && xRight) || (Quad2 && yUp && xLeft) || (Quad3 && yDown && xLeft) || (Quad4 && yDown && xRight))) {
                //Console.WriteLine("quadrant intersection error");
                return double.PositiveInfinity;
            }

            Boolean intersectingX = (xIntersect >= wallX1 - epsilon  && xIntersect <= wallX2 + epsilon) 
                || (xIntersect <= wallX1 + epsilon && xIntersect >= wallX2 - epsilon);
            Boolean intersectingY = (yIntersect >= wallY1 - epsilon && yIntersect <= wallY2 + epsilon) 
                    || (yIntersect <= wallY1 + epsilon && yIntersect >= wallY2 - epsilon);
            // checking if intersection is within wall segment
            if (intersectingX && intersectingY)
            {
                double dist = Math.Sqrt(Math.Pow(xIntersect - x, 2) + Math.Pow(yIntersect - y, 2));
                return dist;
            }
            // ****************** Additional Student Code: End   ************
            //Console.WriteLine("no intersection");
	        return double.PositiveInfinity;
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){

	        double minDist = 6.000;

	        // ****************** Additional Student Code: Start ************

	        // Put code here that loops through segments, calling the
	        // function GetWallDistance.
            int start;
            int end;
            if (navigation.y_est > yOffset - 17)
            {
                start = region1[0];
                end = region1[1];
            }
            else
            {
                start = region2[0];
                end = region2[1];
            }
            for (int i = start; i < end; ++i)
            {
                double currentWallDist = GetWallDistance(x, y, t, i);
                if (currentWallDist < minDist)
                {
                    minDist = currentWallDist;
                }
            }
            // ****************** Additional Student Code: End   ************
           // Console.WriteLine("closest wall: " + minDist);
            return minDist;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        bool CollisionFound(double n1x, double n1y, double n2x, double n2y, double tol){



	        
	        return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
		double dist = 0;

	        return dist;
        }






    }
}
