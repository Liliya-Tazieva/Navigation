using System.Collections.Generic;

namespace Assets.Scripts.PathFinding
{
    public class StraightLinesFromNode
    {
        public List <StraightLine> Lines;

        public StraightLinesFromNode()
        {
            Lines = new List<StraightLine>();
        }

        public StraightLinesFromNode(Node node, List <Node> neighbours)
        {
            Lines = new List<StraightLine>();
            foreach (var neighbour in neighbours)
            {
                Lines.Add( new StraightLine(node, neighbour));
            }
        }

        public StraightLinesFromNode(int x, int y)
        {
            Lines = new List<StraightLine>
            {
                new StraightLine(x, y, Destinations.Up),
                new StraightLine(x, y, Destinations.UpRight),
                new StraightLine(x, y, Destinations.Right),
                new StraightLine(x, y, Destinations.DownRight),
                new StraightLine(x, y, Destinations.Down),
                new StraightLine(x, y, Destinations.DownLeft),
                new StraightLine(x, y, Destinations.Left),
                new StraightLine(x, y, Destinations.UpLeft)
            };
        }

        public static List<Node> ToList(StraightLinesFromNode lines, Node[,] nodesArray)
        {
            var list = new List<Node>();
            foreach (var line in lines.Lines)
            {
                foreach (var point in line.Points)
                {
                    var node = nodesArray[point.X, point.Y];
                    if(node.InformerNode.IsObstacle) continue;
                    list.Add(node);
                }
            }
            return list;
        }
    }
}