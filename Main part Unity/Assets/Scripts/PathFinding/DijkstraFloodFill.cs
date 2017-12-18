using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace Assets.Scripts.PathFinding
{
    class DijkstraFloodFill
    {
        public const int Height = 34;
        public const int Width = 35;

        public Node[,] NodesArray = new Node[Height, Width];

        public DijkstraFloodFill(Node[,] nodes)
        {
            NodesArray = nodes;
        }

        public void Flood()
        {
            
        }
    }
}
