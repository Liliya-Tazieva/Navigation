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

        public BoundingBoxes(Node start, int id)
        {
            StartJP = start;
            BoxID = id;

            BoundJP = new List<Node>();
            RoutesToOtherBB.Add(id, new List<int> {id});
        }

        public static List<BoundingBoxes> FindBoxes(Node[,] nodesArray, int height, int width, List<Node> jumpPoints)
        {
            var boxes = new List<BoundingBoxes>();
            
            return boxes;
        }
        
        public static List<BoundingBoxes> FindPathThroughBoxes(BoundingBoxes startBox,
            BoundingBoxes goalBox, List<BoundingBoxes> boxes)
        {
            var path = new List<BoundingBoxes>();

            return path;
        }

        public void EliminateJPFromBound()
        {
            var mean = BoundJP.Sum(jumpPoint => StartJP.InformerNode.MetricsAStar(jumpPoint.InformerNode));

            mean /= (BoundJP.Count - 1);

            foreach (var jumpPoint in BoundJP)
            {
                if (StartJP.InformerNode.MetricsAStar(jumpPoint.InformerNode) > mean * 2)
                    BoundJP.Remove(jumpPoint);
            }
        }
    }
}