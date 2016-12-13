namespace Assets.Scripts.PathFinding
{
    public class StraightLinesFromNode
    {
        public StraightLine[] Lines;

        public StraightLinesFromNode(int x, int y, Destinations [] destinations)
        {
            Lines = new StraightLine[destinations.Length];
            for (var i=0;i<destinations.Length;++i)
            {
                Lines[i] = new StraightLine(x, y, destinations[i]);
            }
        }

        public StraightLinesFromNode(int x, int y)
        {
            Lines = new StraightLine[8];
            Lines[0] = new StraightLine(x, y, Destinations.Up);
            Lines[1] = new StraightLine(x, y, Destinations.UpRight);
            Lines[2] = new StraightLine(x, y, Destinations.Right);
            Lines[3] = new StraightLine(x, y, Destinations.DownRight);
            Lines[4] = new StraightLine(x, y, Destinations.Down);
            Lines[5] = new StraightLine(x, y, Destinations.DownLeft);
            Lines[6] = new StraightLine(x, y, Destinations.Left);
            Lines[7] = new StraightLine(x, y, Destinations.UpLeft);
        }
    }
}