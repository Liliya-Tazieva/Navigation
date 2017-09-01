using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleApplication2
{
    class NodeArray
    {
        public int height;
        public int widght;
        public Node[,] Array;

        public NodeArray(int h, int w, Node[,] a)
        {
            height = h;
            widght = w;
            Array = a;
        }

        public List<Node> Neighbours(Node current)
        {
            var neighbours = new List<Node>();
            var x = Convert.ToInt32(current.Position.X);
            var y = Convert.ToInt32(current.Position.Y);
            if (x > 0) neighbours.Add(new Node(Array[x - 1, y])); //Left
            if (y < height - 1 && x > 0) neighbours.Add(new Node(Array[x - 1, y + 1])); // Up-Left
            if (y < height - 1) neighbours.Add(new Node(Array[x, y + 1])); //Up
            if (x < widght - 1 && y < height - 1) neighbours.Add(new Node(Array[x + 1, y + 1])); //Up-Right
            if (x < widght - 1) neighbours.Add(new Node(Array[x + 1, y])); //Right
            if (x < widght - 1 && y > 0) neighbours.Add(new Node(Array[x + 1, y - 1])); // Down-Right
            if (y > 0) neighbours.Add(new Node(Array[x, y - 1])); //Down
            if (x > 0 && y > 0) neighbours.Add(new Node(Array[x - 1, y - 1])); // Down-Left

            return neighbours;
        }
    }
}
