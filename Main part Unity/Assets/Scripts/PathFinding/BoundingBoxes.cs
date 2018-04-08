using System.Collections.Generic;
using UnityEngine;

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
            
            /*var histogramX = new int[width];
            var histogramY = new int[height];
            var histogramJpX = new int[width];
            var histogramJpY = new int[height];


            for (var i = 0; i < height; ++i)
            {
                for (var j = 0; j < width; ++j)
                {
                    if (nodesArray[i, j].InformerNode.IsObstacle)
                    {
                        histogramX[j]++;
                        histogramY[i]++;
                    }
                    if (nodesArray[i, j].IsJumpPoint == JPType.Primary)
                    {
                        histogramJpX[j]++;
                        histogramJpY[i]++;
                    }
                }
            }
            
            //Debug
            Debug.Log("HISTOGRAM OBSTACLES X:");
            for (var i = 0; i < width; ++i)
            {
                Debug.Log("bin[" + i + "] " + histogramX[i]);
            }
            Debug.Log("HISTOGRAM OBSTACLES Y:");
            for (var i = 0; i < height; ++i)
            {
                Debug.Log("bin[" + i + "] " + histogramY[i]);
            }
            Debug.Log("HISTOGRAM JP X:");
            for (var i = 0; i < width; ++i)
            {
                Debug.Log("bin[" + i + "] " + histogramJpX[i]);
            }
            Debug.Log("HISTOGRAM JP Y:");
            for (var i = 0; i < height; ++i)
            {
                Debug.Log("bin[" + i + "] " + histogramJpY[i]);
            }

            //Find all local maximums
            for (var i = 1; i < width - 1; ++i)
            {
                if (histogramX[i] > histogramX[i - 1] && histogramX[i] > histogramX[i + 1])
                {
                    Debug.Log("histogramX[" + i + "] " + histogramX[i]+" is a local max");
                }
                if (histogramJpX[i] > histogramJpX[i - 1] && histogramJpX[i] > histogramJpX[i + 1])
                {
                    Debug.Log("histogramJpX[" + i + "] " + histogramJpX[i] + " is a local max");
                }
            }
            for (var i = 1; i < height - 1; ++i)
            {
                if (histogramY[i] > histogramY[i - 1] && histogramY[i] > histogramY[i + 1])
                {
                    Debug.Log("histogramY[" + i + "] " + histogramY[i] + " is a local max");
                }
                if (histogramJpY[i] > histogramJpY[i - 1] && histogramJpY[i] > histogramJpY[i + 1])
                {
                    Debug.Log("histogramJpY[" + i + "] " + histogramJpY[i] + " is a local max");
                }
            }*/

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