using System;
using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;
using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class BoundingBoxes
    {
        public List<Node> BoundJP;
        public Node StartJP;

        public int BoxID;

        public Dictionary<int, List<int>> RoutesToOtherBB = new Dictionary<int, List<int>>();
        
        public List<Node> ConvexHull;

        public BoundingBoxes(Node start, int id)
        {
            StartJP = start;
            BoxID = id;

            BoundJP = new List<Node>();
            RoutesToOtherBB.Add(id, new List<int> {id});
        }

        public static int FindClosestBound(List<Node> jpList, Node node, Node[,] nodesArray, bool forPrimaryJP)
        {
            var closestBound = -1;
            var minDist = 100000f;

            foreach (var jp in jpList)
            {
                var dist = jp.InformerNode.MetricsAStar(node.InformerNode);

                var line = Extensions.BresenhamLineAlgorithm(node, jp);
                
                if (dist < minDist && Extensions.Reachable(line, nodesArray, node.BoundingBox))
                {
                    if(forPrimaryJP && jp.BoundingBox == node.BoundingBox) continue;

                    closestBound = jp.BoundingBox;
                    minDist = dist;
                }
            }

            return closestBound;
        }

        private static double Cross(Node O, Node A, Node B)
        {
            return (A.X() - O.X()) * (B.Y() - O.Y()) - (A.Y() - O.Y()) * (B.X() - O.X());
        }

        public void FilterPointsInBound(Node[,] nodesArray)
        {
            var intersection = new Dictionary<Vector3, int>();
            var mean = 0f;
            foreach (var jp in BoundJP)
            {
                intersection.Add(jp.Position, jp.VisibleJP.Intersect(BoundJP).Count());
                mean += intersection[jp.Position];
            }

            mean /= intersection.Count;

            BoundJP.RemoveAll(arg => intersection[arg.Position] < mean);
        }

        //Monotone Chain algorithm, a.k.a. Andrew's Algorithm
        public void FindConvexHull ()
        {
            int n = BoundJP.Count, k = 0;
            ConvexHull = new List<Node>(new Node[2 * n]);

            if (BoundJP.Count <= 3)
            {
                ConvexHull = BoundJP;
                return;
            }

            
            BoundJP.Sort((a, b) =>
                a.X() == b.X() ? a.Y().CompareTo(b.Y()) : a.X().CompareTo(b.X()));

            // Build lower hull
            for (var i = 0; i < n; ++i)
            {
                while (k >= 2 && Cross(ConvexHull[k - 2], ConvexHull[k - 1], BoundJP[i]) <= 0)
                    k--;
                ConvexHull[k++] = BoundJP[i];
            }

            // Build upper hull
            for (int i = n - 2, t = k + 1; i >= 0; i--)
            {
                while (k >= t && Cross(ConvexHull[k - 2], ConvexHull[k - 1], BoundJP[i]) <= 0)
                    k--;
                ConvexHull[k++] = BoundJP[i];
            }

            ConvexHull = ConvexHull.Take(k - 1).ToList();
        }

        public bool IsInsideBb(Node point)
        {
            var inside = false;
            
            if (ConvexHull.Count < 3)
            {
                Debug.LogWarning("Convex hull contains less than 3 points");
                return false;
            }
            
            var oldPoint = new Node(ConvexHull[ConvexHull.Count - 1]);
            
            foreach (var pointInHull in ConvexHull)
            {
                var newPoint = new Node(pointInHull);

                Node point1;
                Node point2;
                if (newPoint.X() > oldPoint.X())
                {
                    point1 = oldPoint;
                    point2 = newPoint;
                }
                else
                {
                    point1 = newPoint;
                    point2 = oldPoint;
                }


                if (newPoint.X() < point.X() == point.X() <= oldPoint.X()
                    && (point.Y() - point1.Y()) * (point2.X() - point1.X()) < (point2.Y() - point1.Y()) * (point.X() - point1.X()))
                    inside = !inside;
                
                oldPoint = newPoint;
            }
            
            return inside;
        }
    }
}