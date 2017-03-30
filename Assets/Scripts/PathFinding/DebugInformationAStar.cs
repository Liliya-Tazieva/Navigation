using System.Collections.Generic;

namespace Assets.Scripts.PathFinding {
    public class DebugInformationAlgorithm
    {
        public List<Informer> FinalPath = new List<Informer>();
        public List<Informer> FinalPathJPS = new List<Informer>();
        public Informer From;
        public List<Node> Observed = new List<Node>();
        public Informer To;
        public List<Node> LinesFromFinish = new List<Node>();
        public List<Node> CrossPoints = new List<Node>();
        public bool Destroy = true;
    }
}