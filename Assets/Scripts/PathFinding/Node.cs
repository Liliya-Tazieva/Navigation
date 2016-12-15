using AForge.Math;
using Vector3 = UnityEngine.Vector3;

namespace Assets.Scripts.PathFinding {
    public enum NodeState {
        Undiscovered = 2,
        Discovered = 0,
        Processed = 1
    }

    public class Node {

        public int[,] NormMatrix = new int[3, 3];
        public bool JumpPoint;
        public bool TargetJP;
        public Vector3 Position;

        public readonly Informer InformerNode;
        public NodeState Visited;
        public float Distance;
        public Destinations DestinationFromPrevious;
        public Destinations DestinationToFinish;

        public Node(Informer i, NodeState v) {
            InformerNode = i;
            Visited = v;
            Distance = 0;
            Position = i.transform.position;
            
            for (var k = 0; k < 3; ++k)
            {
                for (var j = 0; j<3; ++j)
                {
                    NormMatrix[k, j] = 0;
                }
            }
			JumpPoint = false;
            TargetJP = false;
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