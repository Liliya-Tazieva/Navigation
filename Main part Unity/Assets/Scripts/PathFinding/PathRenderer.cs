using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.PathFinding {
    public enum Show {
        Observed = 0,
        Path = 1,
        From = 2,
        To = 3,
		Line = 4,
        CrossPoint = 5,
        PathJPS = 6
    }

    public class PathRenderer: MonoBehaviour {
        private class RendererUseInformer {
            public Color DefaultColor;
            public uint UseCount;
        }

        private static Dictionary<Renderer, RendererUseInformer> _defaultColors = new Dictionary<Renderer, RendererUseInformer>();
        private readonly HashSet<Renderer> _usedRenderers = new HashSet<Renderer>();

        public DebugInformationAlgorithm DebugInformation;

        public static void MapRebuild() {
            _defaultColors = new Dictionary<Renderer, RendererUseInformer>();
        }

        [UsedImplicitly]
        private IEnumerator Start() {
            yield return StartCoroutine( "RendererPath" );
        }

        [UsedImplicitly]
        public void OnDestroy()
        {
            if (DebugInformation!=null && DebugInformation.Destroy)
            {
                _usedRenderers.ForEach(e => _defaultColors[e].UseCount--);

                if (_defaultColors != null)
                {
                    _defaultColors
                        .Where(pair => pair.Value.UseCount == 0)
                        .ForEach(pair =>
                        {
                            if (pair.Key != null && (pair.Key.material.GetColor("_Color") != Color.cyan
                                                     && pair.Key.material.GetColor("_Color") != Color.magenta))
                            {
                                pair.Key.material.SetColor("_Color", pair.Value.DefaultColor);
                            }
                        });
                }
            }
        }

        [UsedImplicitly]
        public IEnumerator RendererPath() {
            if (DebugInformation != null) {
                yield return AStarDebug(DebugInformation.From, Show.From);
                yield return AStarDebug(DebugInformation.To, Show.To);
                foreach (var informer in DebugInformation.LinesFromFinish)
                {
                    if (informer.InformerNode != DebugInformation.From && informer.InformerNode != DebugInformation.To)
                    {
                        yield return AStarDebug(informer.InformerNode, Show.Line);
                    }
                }
                foreach (var informer in DebugInformation.CrossPoints)
                {
                    if (informer.InformerNode != DebugInformation.From && informer.InformerNode != DebugInformation.To)
                    {
                        yield return AStarDebug(informer.InformerNode, Show.CrossPoint);
                    }
                }
                foreach (var informer in DebugInformation.Observed) {
                    if (informer.InformerNode != DebugInformation.From && informer.InformerNode != DebugInformation.To) {
                        yield return AStarDebug(informer.InformerNode, Show.Observed);
                    }
                }
                foreach (var node in DebugInformation.FinalPath) {
                    if (node.InformerNode != DebugInformation.From && node.InformerNode != DebugInformation.To) {
                        yield return AStarDebug(node.InformerNode, Show.Path);
                    }
                }
                foreach (var node in DebugInformation.FinalPathJPS)
                {
                    if (node.InformerNode != DebugInformation.From && node.InformerNode != DebugInformation.To)
                    {
                        yield return AStarDebug(node.InformerNode, Show.PathJPS);
                    }
                }
            } else yield return null;
            Destroy(this);
        }

        private IEnumerator AStarDebug(Component informer, Show show)
        {
            if (informer == null) yield break;
            var component = informer.GetComponent<Renderer>();


            RendererUseInformer rendererInformer;

            if (!_defaultColors.TryGetValue(component, out rendererInformer))
            {
                var color = component.material.GetColor("_Color");

                rendererInformer = new RendererUseInformer()
                {
                    DefaultColor = color,
                    UseCount = 1
                };
                _usedRenderers.Add(component);
                _defaultColors.Add(component, rendererInformer);
            }
            else if (_usedRenderers.Add(component))
            {
                rendererInformer.UseCount++;
            }

            if (show == Show.Observed)
            {
                component.material.SetColor("_Color", Color.green);
            }
            else if (show == Show.Path)
            {
                component.material.SetColor("_Color", Color.yellow);
            }
            else if (show == Show.From)
            {
                component.material.SetColor("_Color", Color.cyan);
            }
            else if (show == Show.To)
            {
                component.material.SetColor("_Color", Color.magenta);
            }
            else if (show == Show.Line)
            {
                component.material.SetColor("_Color", Color.gray);
            }
            else if (show == Show.CrossPoint)
            {
                component.material.SetColor("_Color", Color.white);
            }
            else
            {
                component.material.SetColor("_Color", Color.red);
            }
            yield return new WaitForSeconds(.01f);
        }
    }
}