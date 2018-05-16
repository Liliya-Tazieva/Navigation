using System.Collections.Generic;

namespace Assets.Scripts.PathFinding {
    public class DebugInformationAlgorithm
    {
        public List<Node> FinalPath = new List<Node>();
        public List<Node> FinalPathJPS = new List<Node>();
        public Informer From;
        public List<Node> Observed = new List<Node>();
        public Informer To;
        public List<Node> LinesFromFinish = new List<Node>();
        public List<Node> CrossPoints = new List<Node>();
        public bool Destroy = true;
    }
}