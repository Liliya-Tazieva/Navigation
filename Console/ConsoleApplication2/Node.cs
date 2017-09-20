using System;
using System.Numerics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace ConsoleApplication2
{
    public enum NodeState
    {
        Undiscovered = 2,
        Discovered = 0,
        Processed = 1
    }

    public enum JPType
    {
        Default = 2,
        Diagonal = 1,
        Primary = 0
    }

    public enum Destinations
    {
        Default = 0,
        UpRight = 1,
        DownRight = 2,
        UpLeft = 3,
        DownLeft = 4,
        Right = 5,
        Left = 6,
        Up = 7,
        Down = 8
    }

    class Node
    {
        public int[,] NormMatrix = new int[3, 3];
        public JPType IsJumpPoint;
        public bool TargetJP;
        public Vector2 Position;
        public readonly bool IsObstacle;
        public NodeState Visited;
        public float DistanceToGoal;
        public float DistanceToStart;
        public Destinations DestinationFromPrevious;
        public Destinations DestinationToFinish;
        public Node Parent;

        public Node(int i, int j, char symbol)
        {
            Visited = NodeState.Undiscovered;
            DistanceToGoal = 0;
            DistanceToStart = 0;
            Position.X = i;
            Position.Y = j;
            DestinationFromPrevious = Destinations.Default;
            DestinationToFinish = Destinations.Default;
            IsJumpPoint = JPType.Default;
            TargetJP = false;
            Parent = null;

            for (var k = 0; k < 3; ++k)
            {
                for (var h = 0; h < 3; ++h)
                {
                    NormMatrix[k, h] = 0;
                }
            }

            switch (symbol)
            {
                case 'T':
                    IsObstacle = true;
                    break;
                default:
                    IsObstacle = false;
                    break;
            }
        }

        public Node(Node n)
        {
            if (n != null)
            {
                IsObstacle = n.IsObstacle;
                Visited = n.Visited;
                DistanceToGoal = n.DistanceToGoal;
                DistanceToStart = n.DistanceToStart;
                Position = n.Position;
                DestinationToFinish = n.DestinationToFinish;
                DestinationFromPrevious = n.DestinationFromPrevious;
                NormMatrix = n.NormMatrix;
                IsJumpPoint = n.IsJumpPoint;
                TargetJP = n.TargetJP;
                Parent = n.Parent;
            }
        }

        public Node(Node parent, Node node, Destinations destinationFromParent)
        {
            IsObstacle = node.IsObstacle;
            Visited = node.Visited;
            DistanceToGoal = sizeof(float);
            Position = node.Position;
            DestinationToFinish = node.DestinationToFinish;
            DistanceToStart = MetricsAStar(parent,node);
            DestinationFromPrevious = destinationFromParent;
            NormMatrix = node.NormMatrix;
            IsJumpPoint = node.IsJumpPoint;
            TargetJP = node.TargetJP;
            Parent = parent;
        }

        public static float MetricsAStar(Node from, Node to)
        {
            return Vector2.DistanceSquared(from.Position, to.Position);
        }
    }
}
