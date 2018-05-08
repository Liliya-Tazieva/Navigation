using System.Collections.Generic;
using AForge.Math;
using Vector3 = UnityEngine.Vector3;

namespace Assets.Scripts.PathFinding {
    public enum NodeState {
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

    public class Node {

        public int[,] NormMatrix = new int[3, 3];
        public JPType IsJumpPoint;
        public bool TargetJP;
        public Vector3 Position;

        public readonly Informer InformerNode;
        public NodeState Visited;
        public float DistanceToFinish;
        public float DistanceToStart;
        public Destinations DestinationFromPrevious;
        public Destinations DestinationToFinish;

        public List<Node> VisibleJP;
        public int BoundingBox;

        public Node(Informer i, NodeState v) {
            InformerNode = i;
            Visited = v;
            DistanceToFinish = 0;
            DistanceToStart = 0;
            Position = i.transform.position;
            DestinationFromPrevious = Destinations.Default;
            DestinationToFinish = Destinations.Default;


            for (var k = 0; k < 3; ++k)
            {
                for (var j = 0; j<3; ++j)
                {
                    NormMatrix[k, j] = 0;
                }
            }
            
            IsJumpPoint = JPType.Default;
            TargetJP = false;

            VisibleJP = new List<Node>();
            BoundingBox = -1;
        }

        public Node(Node n)
        {
            if (n != null)
            {
                InformerNode = n.InformerNode;
                Visited = n.Visited;
                DistanceToFinish = n.DistanceToFinish;
                DistanceToStart = n.DistanceToStart;
                Position = n.Position;
                DestinationToFinish = n.DestinationToFinish;
                DestinationFromPrevious = n.DestinationFromPrevious;

                NormMatrix = n.NormMatrix;
                IsJumpPoint = n.IsJumpPoint;
                TargetJP = n.TargetJP;

                VisibleJP = n.VisibleJP;
                BoundingBox = n.BoundingBox;
            }
        }


        public void RestartNode()
        {
            for (var k = 0; k < 3; ++k)
            {
                for (var j = 0; j < 3; ++j)
                {
                    NormMatrix[k, j] = 0;
                }
            }

            IsJumpPoint = JPType.Default;
            TargetJP = false;

            VisibleJP.Clear();
            BoundingBox = -1;
        }

        public int X()
        {
            return (int)InformerNode.transform.position.x/3;
        }
        public int Y()
        {
            return (int)InformerNode.transform.position.z/3;
        }
    }
}