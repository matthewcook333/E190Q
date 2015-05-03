using System;
using System.Collections.Generic;
using System.IO;
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

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        public Map()
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:

            // count number of lines in CSV file
            /*
            numMapSegments = File.ReadAllLines(@"C:\Users\mattcook\Desktop\extended_map3b.csv").Length;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            segmentSizes = new double[numMapSegments];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];*/

            // open CSV file
            //var reader = new StreamReader(File.OpenRead(@"C:\Users\mattcook\Desktop\extended_map3b.csv"));
            //int r = 0;
            // read line by line
            //while (!reader.EndOfStream)
            //{
            //    var line = reader.ReadLine();
            //    var values = line.Split(',');
            //    mapSegmentCorners[r, 0, 0] = Convert.ToDouble(values[0]);
            //    mapSegmentCorners[r, 0, 1] = Convert.ToDouble(values[1]);
            //    mapSegmentCorners[r, 1, 0] = Convert.ToDouble(values[2]);
            //    mapSegmentCorners[r, 1, 1] = Convert.ToDouble(values[3]);
            //    if (r >= 0 && r < numMapSegments)
            //    {
            //        Console.WriteLine("mapSegmentCorners[" + r + ", 0, 0] = " + mapSegmentCorners[r, 0, 0]);
            //        Console.WriteLine("mapSegmentCorners[" + r + ", 0, 1] = " + mapSegmentCorners[r, 0, 1]);
            //        Console.WriteLine("mapSegmentCorners[" + r + ", 1, 0] = " + mapSegmentCorners[r, 1, 0]);
            //        Console.WriteLine("mapSegmentCorners[" + r + ", 1, 1] = " + mapSegmentCorners[r, 1, 1]);
            //    }
            //    r++;
            //}
            //reader.Close();


            
	        numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
	        mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

	        mapSegmentCorners[1,0,0] = -3.55/2;
	        mapSegmentCorners[1,0,1] = 0.0;
	        mapSegmentCorners[1,1,0] = -3.55/2;
	        mapSegmentCorners[1,1,1] = -2.74;

	        mapSegmentCorners[2,0,0] = 3.55/2;
	        mapSegmentCorners[2,0,1] = 0.0;
	        mapSegmentCorners[2,1,0] = 3.55/2;
	        mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
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
        double GetWallDistance(double x, double y, double t, int segment)
        {
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
            Boolean Quad2 = (yIntersect >= y && xIntersect < x);
            Boolean Quad3 = (yIntersect <= y && xIntersect < x);
            Boolean Quad4 = (yIntersect < y && xIntersect >= x);

            // checking if intersection is actually in laser sight rather than behind
            if (!((Quad1 && yUp && xRight) || (Quad2 && yUp && xLeft) || (Quad3 && yDown && xLeft) || (Quad4 && yDown && xRight)))
            {
                //Console.WriteLine("quadrant intersection error");
                return double.PositiveInfinity;
            }

            Boolean intersectingX = (xIntersect >= wallX1 - epsilon && xIntersect <= wallX2 + epsilon)
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

        public double GetClosestWallDistance(double x, double y, double t)
        {

            double minDist = 6.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.
            for (int i = 0; i < numMapSegments; ++i)
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

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol)
        {


            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }
            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
            // Set wall vars
            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];
            double dist = 9999;

            // Put code here to calculated dist.
            // Calculate slope and intercept
            double angleSegmentPerpendicular = Math.PI / 2 + Math.Atan((Y2 - Y1) / (0.000001 + X2 - X1));
            double m = Math.Tan(angleSegmentPerpendicular);
            double b = y - m * x;

            // Get line intersection
            double x_intersect = (b - intercepts[segment]) / (slopes[segment] - m);
            double y_intersect = m * x_intersect + b;

            // Check for horiz/vert slopes
            if (Math.Abs(Y2 - Y1) < 0.001)
                y_intersect = Y1;
            if (Math.Abs(X2 - X1) < 0.001)
                x_intersect = X1;


            // Check to see if intersection LIES within segment
            double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect - X1, 2) + Math.Pow(y_intersect - Y1, 2));
            double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect - X2, 2) + Math.Pow(y_intersect - Y2, 2));
            if (dist_intersect_corner1 <= (segmentSizes[segment] + tol) && dist_intersect_corner2 <= (segmentSizes[segment] + tol))
            {
                dist = Math.Sqrt(Math.Pow(x - x_intersect, 2) + Math.Pow(y - y_intersect, 2));
            }

            // Check for distance to corners (for case where no intersection with segment
            double dist_point_corner1 = Math.Sqrt(Math.Pow(x - X1, 2) + Math.Pow(y - Y1, 2));
            double dist_point_corner2 = Math.Sqrt(Math.Pow(x - X2, 2) + Math.Pow(y - Y2, 2));
            dist = Math.Min(dist, dist_point_corner1);
            dist = Math.Min(dist, dist_point_corner2);

            return dist;
        }






    }
}
