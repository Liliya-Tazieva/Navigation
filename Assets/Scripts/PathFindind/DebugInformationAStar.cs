using System.Collections.Generic;

namespace Assets.Scripts.PathFinding {
    public class DebugInformationAStar {
        public List<Informer> FinalPath = new List<Informer>();
        public Informer From;
        public List<Node> Observed = new List<Node>();
        public Informer To;
    }
}