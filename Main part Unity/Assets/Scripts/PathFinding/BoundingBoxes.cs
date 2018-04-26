using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;
using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class BoundingBoxes
    {
        public int MinX;
        public int MaxX;
        public int MinY;
        public int MaxY;

        public List<Node> BoundJP;
        public Node StartJP;

        public int BoxID;

        public BoundingBoxes(Node start, int id)
        {
            StartJP = start;
            BoxID = id;

            BoundJP = new List<Node>();
        }

        public static List<BoundingBoxes> FindBoxes(Node[,] nodesArray, int height, int width, List<Node> jumpPoints)
        {
            var boxes = new List<BoundingBoxes>();
            
            return boxes;
        }

        public static bool InBox(Node node, BoundingBoxes box)
        {
            return box.MinX > node.X() && box.MaxX < node.X()
                   && box.MinY > node.Y() && box.MaxY > node.Y();
        }

        public static BoundingBoxes BelongToBox(Node node, List<BoundingBoxes> boxes)
        {
            return boxes.Find(arg => arg.MinX > node.X() && arg.MaxX < node.X()
                                     && arg.MinY > node.Y() && arg.MaxY > node.Y());
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