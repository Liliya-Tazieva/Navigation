namespace Assets.Scripts.PathFinding {
    public enum NodeState {
        Undiscovered = 2,
        Discovered = 0,
        Processed = 1
    }

    public class Node {
		public bool JumpPoint;
        public int[,] NormMatrix = new int[3,3];

        public readonly Informer InformerNode;
        public NodeState Visited;
        public float Distance;
        public Destinations DestinationFromPrevious;

        public Node(Informer i, NodeState v) {
            InformerNode = i;
            Visited = v;
            Distance = 0;
            DestinationFromPrevious = Destinations.Default; 
            for (var k = 0; k < 3; ++k)
            {
                for (var j = 0; j<3; ++j)
                {
                    NormMatrix[k, j] = 0;
                }
            }
			JumpPoint = false;
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