using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;

namespace Assets.Scripts.PathFinding
{
    public class BoundingBoxes
    {
        public List<Node> BoundJP;
        public Node StartJP;

        public int BoxID;

        public Dictionary<int, List<int>> RoutesToOtherBB = new Dictionary<int, List<int>>();

        public BoundingBoxes(Node start, int id)
        {
            StartJP = start;
            BoxID = id;

            BoundJP = new List<Node>();
            RoutesToOtherBB.Add(id, new List<int> {id});
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
    }
}