using System.Collections.Generic;

namespace Assets.Scripts.PathFinding
{
    public class StraightLinesFromNode
    {
        public List <StraightLine> Lines;

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
    }
}