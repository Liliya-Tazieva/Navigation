namespace Assets.Scripts.PathFinding {
    public enum NodeState {
        Undiscovered = 2,
        Discovered = 0,
        Processed = 1
    }

    public class Node {
        public readonly Informer InformerNode;
        public NodeState Visited;
        public float Distance;

        public Node(Informer i, NodeState v) {
            InformerNode = i;
            Visited = v;
            Distance = 0;
        }
    }
}