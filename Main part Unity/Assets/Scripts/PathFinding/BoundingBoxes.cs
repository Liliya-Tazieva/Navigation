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

        public List<Node> AllPoints;
        public List<Node> ConvexHull;

        public BoundingBoxes(Node start, int id)
        {
            StartJP = start;
            BoxID = id;

            BoundJP = new List<Node>();
            RoutesToOtherBB.Add(id, new List<int> {id});

            AllPoints = new List<Node>();
        }

        public static int FindClosestBound(List<Node> jpList, Node node, Node[,] nodesArray)
        {
            var closestJp = jpList[0];
            var minDist = 100000;

            foreach (var jp in jpList)
            {
                var dist = jp.InformerNode.MetricsAStar(node.InformerNode);

                if (dist < minDist && Extensions.Reachable(node, jp, nodesArray)) closestJp = jp;
            }

            return closestJp.BoundingBox;
        }

        private static double Cross(Node O, Node A, Node B)
        {
            return (A.X() - O.X()) * (B.Y() - O.Y()) - (A.Y() - O.Y()) * (B.X() - O.X());
        }

        //Monotone Chain algorithm, a.k.a. Andrew's Algorithm
        public void FindConvexHull()
        {
            if (AllPoints == null)
            {
                Debug.LogWarning("Convex Hull can't be found with no points");
                return;
            }

            if (AllPoints.Count() <= 1)
                ConvexHull = AllPoints;

            int n = AllPoints.Count(), k = 0;

            ConvexHull = new List<Node>(new Node[2 * n]);

            AllPoints.Sort((a, b) =>
                a.X() == b.X() ? a.Y().CompareTo(b.Y()) : a.X().CompareTo(b.X()));

            // Build lower hull
            for (var i = 0; i < n; ++i)
            {
                while (k >= 2 && Cross(ConvexHull[k - 2], ConvexHull[k - 1], AllPoints[i]) <= 0)
                    k--;
                ConvexHull[k++] = AllPoints[i];
            }

            // Build upper hull
            for (int i = n - 2, t = k + 1; i >= 0; i--)
            {
                while (k >= t && Cross(ConvexHull[k - 2], ConvexHull[k - 1], AllPoints[i]) <= 0)
                    k--;
                ConvexHull[k++] = AllPoints[i];
            }

            ConvexHull = ConvexHull.Take(k - 1).ToList();

            AllPoints.Clear();
        }
    }
}