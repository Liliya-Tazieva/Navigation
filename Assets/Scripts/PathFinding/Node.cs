namespace Assets.Scripts.PathFinding {
    public enum NodeState {
        Undiscovered = 2,
        Discovered = 0,
        Processed = 1
    }

	public enum Destination {
		Default = 0,
		Up_right = 1,
		Down_right = 2,
		Up_left = 3,
		Down_left = 4,
		Right = 5,
		Left = 6,
		Up = 7,
		Down = 8
	}

    public class Node {
        public readonly Informer InformerNode;
        public NodeState Visited;
        public float Distance;
		public Destination DestinationToStart;

        public Node(Informer i, NodeState v) {
            InformerNode = i;
            Visited = v;
            Distance = 0;
			DestinationToStart = 0;
        }
    }
}