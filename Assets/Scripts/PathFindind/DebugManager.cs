using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.PathFinding {
    public class DebugManager : MonoBehaviour {
        private readonly List<DebugInformationAStar> _paths = new List<DebugInformationAStar>();

        public void AddPath(DebugInformationAStar dbi) {
            _paths.Add(dbi);
        }

        [UsedImplicitly]
        private void Update() {
            if (_paths.Any()) {
                foreach (var path in _paths) {
                    var component = gameObject.AddComponent<PathRenderer>();
                    component.DebugInformation = path;
                }
                _paths.Clear();
            }
        }
    }
}