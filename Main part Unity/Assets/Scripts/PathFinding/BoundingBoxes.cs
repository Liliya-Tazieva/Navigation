using System.Collections.Generic;

namespace Assets.Scripts.PathFinding
{
    public class BoundingBoxes
    {
        public int MinX;
        public int MaxX;
        public int MinY;
        public int MaxY;

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
    }
}