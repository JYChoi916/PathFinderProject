using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Tilemaps;
using System;

public enum PathFinderType
{
    Dijkstra,
    AStar,
    JPS
}

public class PathFinder : MonoBehaviour
{
    public Tilemap tileMap;     // 타일맵
    public Tilemap pathTileMap; // 경로 타일맵
    public TileBase floorTile;  // 바닥 타일
    public TileBase wallTile;   // 벽 타일
    public TileBase pathTile;   // 경로 타일
    public PathFinderType pathFinderType = PathFinderType.Dijkstra;     // 길찾기 타입

    private LineRenderer pathLine;    // 경로 라인 렌더러

    public bool allowDiagonal = false; // 대각선 이동 허용 옵션

    private Dictionary<Vector2Int, Node> nodes = new Dictionary<Vector2Int, Node>();    // 모든 노드 저장
    private HashSet<Node> openSet = new HashSet<Node>();                                // 방문하지 않은 노드들
    private HashSet<Node> closedSet = new HashSet<Node>();                              // 방문한 노드들

    private List<Vector2Int> currentPath = new List<Vector2Int>();                   // 현재 경로 저장


    private readonly Vector2Int[] diagonalDirections = new Vector2Int[]
    {
        new Vector2Int(1, 1),    // 우상
        new Vector2Int(-1, 1),   // 좌상
        new Vector2Int(1, -1),   // 우하
        new Vector2Int(-1, -1)   // 좌하
    };

    void Start()
    {
        InitializeNodes();
        InitializeLineRenderer();
    }  

    /// <summary>
    /// 모든 타일 맵에 대하여 1:1로 노드 생성 초기화
    /// </summary>
    void InitializeNodes() 
    {
        BoundsInt bounds = tileMap.cellBounds;
        for (int x = bounds.min.x; x < bounds.max.x; x++)
        {
            for (int y = bounds.min.y; y < bounds.max.y; y++)
            {
                // 각 셀마다 노드를 생성
                Vector2Int pos = new Vector2Int(x, y);
                Node node = new Node(pos);

                // 현재타일의 벽인지 확인
                Vector3Int tilePos = new Vector3Int(x, y, 0);
                if (tileMap.GetTile(tilePos) == wallTile)
                {
                    node.isWall = true;
                }
                // 노드를 Dictionary nodes에 저장 
                nodes[pos] = node;
            }
        }
    }

    /// <summary>
    /// LineRenderer 초기화
    /// </summary>
    private void InitializeLineRenderer()
    {
        // LineRenderer 컴포넌트가 없다면 추가
        pathLine = GetComponent<LineRenderer>();
        if (pathLine == null)
            pathLine = gameObject.AddComponent<LineRenderer>();

        // LineRenderer 기본 설정
        pathLine.startWidth = 0.1f;
        pathLine.endWidth = 0.1f;
        pathLine.positionCount = 0;
        
        // 라인 머테리얼 설정 (기본 머테리얼 사용)
        pathLine.material = new Material(Shader.Find("Sprites/Default"));
        pathLine.startColor = Color.yellow;
        pathLine.endColor = Color.yellow;
    }

    /// <summary>
    /// 방문하지 않은 노드들 중 가장 가까운 노드 가져오기
    /// </summary>
    /// <returns></returns>
    Node GetClosestUnvisitedNode()
    {
        // 반환할 가장 가까운 노드
        Node closestNode = null;

        // 가장 작은 총 비용을 가진 노드의 총 비용
        float minTotalCost = Mathf.Infinity;

        // 방문하지 않은 모든 노드에 대하여
        foreach (Node node in openSet)
        {
            // 총 비용 계산 (F = G + H)
            float totalCost = node.distance + node.heuristic;

            // 총 비용이 가장 작은 노드를 선택
            if (totalCost < minTotalCost || (totalCost == minTotalCost && node.heuristic < closestNode?.heuristic))
            {
                minTotalCost = totalCost;
                closestNode = node;
            }
        }

        return closestNode;
    }

    /// <summary>
    /// 갈 수 있는 이웃 노드들 가져오기
    /// </summary>
    /// <param name="currentPosition"></param>
    /// <returns></returns>
    List<Vector2Int> GetNeighbors(Vector2Int currentPosition)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>{
            currentPosition + Vector2Int.up,        // 위
            currentPosition + Vector2Int.down,      // 아래
            currentPosition + Vector2Int.left,      // 왼쪽
            currentPosition + Vector2Int.right      // 오른쪽
        };

        // 대각선 이동이 허용된 경우 대각선 방향도 추가
        if (allowDiagonal)
        {
            foreach (Vector2Int dir in diagonalDirections)
            {
                Vector2Int diagonalPos = currentPosition + dir;
                
                // 수평, 수직 위치에 벽이 있는지 확인
                Vector2Int horizontal = new Vector2Int(dir.x, 0);
                Vector2Int vertical = new Vector2Int(0, dir.y);
                
                bool isWallH = nodes.ContainsKey(currentPosition + horizontal) && 
                    nodes[currentPosition + horizontal].isWall;
                bool isWallV = nodes.ContainsKey(currentPosition + vertical) &&
                    nodes[currentPosition + vertical].isWall;

                // 대각선 이동 시 벽 체크 (대각선으로 벽을 통과하지 못하도록)
                if (isWallH && isWallV)
                    continue;

                neighbors.Add(diagonalPos);
            }
        }

        return neighbors;
    }

    /// <summary>
    /// 거리 계산
    /// </summary>
    int GetDistance(Vector2Int from, Vector2Int to)
    {
        Vector2Int diff = to - from;
        if (Mathf.Abs(diff.x) == 1 && Mathf.Abs(diff.y) == 1)
        {
            return 14;
        }

        return 10;
    }

    int nodesVisited;
    public void FindPath(Vector2Int start, Vector2Int end)
    {
        // 방문하지 않은 노드 클리어
        openSet.Clear();

        // 방문한 노드 클리어
        closedSet.Clear();

        nodesVisited = 0; // 카운터 초기화

        // 시작 노드에 대하여 모든 노드의 거리 초기화
        foreach (Node node in nodes.Values)
        {
            node.distance = Mathf.Infinity;
            node.previous = null;
            // 방문하지 않은 노드 초기화
            if (pathFinderType != PathFinderType.JPS)
                openSet.Add(node);
        }

        nodes[start].distance = 0;
        if (pathFinderType == PathFinderType.JPS)
            openSet.Add(nodes[start]);

        List<Vector2Int> newPath = new List<Vector2Int>();
        switch(pathFinderType)
        {
            case PathFinderType.Dijkstra:
                newPath = FindPathDijkstra(start, end);
                break;
            case PathFinderType.AStar:
                newPath = FindPathAStar(start, end);
                break;
            case PathFinderType.JPS:
                newPath = FindPathJPS(start, end);
                break;
        }

        VisualizePath(newPath);
        currentPath = newPath;

        Debug.Log($"방문한 노드의 수 : {nodesVisited}");
    }

    /// <summary>
    /// 다익스트라 알고리즘을 이용한 길찾기
    /// </summary>
    /// <param name="start"></param>
    /// <param name="end"></param>
    /// <returns></returns>
    List<Vector2Int> FindPathDijkstra(Vector2Int start, Vector2Int end)
    {
        // 모든 방문하지 않은 노드들에 대하여 시작지점으로부터 거리 계산
        while (openSet.Count > 0)
        {
            // 미방문 노드중 가장 가까운 노드를 현재 노드로 선택 (첫 반복에서는 시작노드, 이유는 시작노드만 거리가 0이므로)
            Node currentNode = GetClosestUnvisitedNode();
            nodesVisited++;

            // 현재 노드를 못가져왔거나 도착점에 도달했다면 탐색 종료
            if (currentNode == null || currentNode.position == end)
            {
                break;
            }

            // 현재 노드를 방문한 노드 HashSet에 추가
            closedSet.Add(currentNode);

            // 현재 노드를 방문하지 않은 노드 HashSet에서 제거
            openSet.Remove(currentNode);

            // 현재 노드를 기준으로 이웃들의 거리를 갱신
            foreach (Vector2Int neighborPos in GetNeighbors(currentNode.position))
            {
                // 이웃 노드가 맵 밖이거나 벽이거나 이미 방문한 노드라면 스킵
                if (!nodes.ContainsKey(neighborPos) || nodes[neighborPos].isWall || closedSet.Contains(nodes[neighborPos]))
                {
                    continue;
                }

                Node neighbor = nodes[neighborPos];

                // 현재 노드에서 이웃 노드까지 거리를 현재 노드의 거리 + 1로 설정
                float newDistance = currentNode.distance + GetDistance(currentNode.position, neighborPos);

                if (newDistance < neighbor.distance)
                {
                    neighbor.distance = newDistance;
                    neighbor.previous = currentNode;
                }
            }
        }

        // 최종 경로 생성해서 반환
        return BuildPath(end);
    }

    /// <summary>
    /// 두 지점간의 휴리스틱 값(예상 거리) 계산
    /// </summary>
    int CalculateHeuristic(Vector2Int current, Vector2Int end)
    {
        if (allowDiagonal)
        {
            // 대각선 이동이 가능한 경우의 휴리스틱
            int dx = Mathf.Abs(current.x - end.x);
            int dy = Mathf.Abs(current.y - end.y);
            return 14 * Mathf.Min(dx, dy) + 10 * Mathf.Abs(dx - dy);
        }
        else
        {
            // 기존의 맨해튼 거리
            return Mathf.Abs(current.x - end.x) + Mathf.Abs(current.y - end.y) * 10;
        }
    }


    /// <summary>
    /// A* 알고리즘을 이용한 길찾기
    /// </summary>
    List<Vector2Int> FindPathAStar(Vector2Int start, Vector2Int end)
    {
        // 시작노드의 휴리스틱 값 설정
        nodes[start].heuristic = CalculateHeuristic(start, end);

        // 모든 방문하지 않은 노드들에 대해 총 비용 계산 (F = G + H)
        while (openSet.Count > 0)
        {
            // 미방문 노드중 총 비용(F)이 가장 작은 노드를 현재 노드로 선택
            Node currentNode = GetClosestUnvisitedNode();
            nodesVisited++;

            // 현재노드를 못가져왔거나 도착점에 도달했다면 탐색 종료
            if (currentNode == null || currentNode.position == end)
            {
                break;
            }

            // 현재 노드를 방문한 노드 HashSet에 추가
            closedSet.Add(currentNode);

            // 현재 노드를 방문하지 않은 노드 HashSet에서 제거
            openSet.Remove(currentNode);

            // 현재 노드를 기준으로 이웃들을 검사
            foreach(Vector2Int neighborPos in GetNeighbors(currentNode.position))
            {
                // 이웃 노드가 맵 밖이거나 벽이거나 이미 방문한 노드라면 스킵
                if (!nodes.ContainsKey(neighborPos) || nodes[neighborPos].isWall || closedSet.Contains(nodes[neighborPos]))
                {
                    continue;
                }

                Node neighbor = nodes[neighborPos];

                // 현재 노드에서 이웃 노드까지의 거리 계산
                float newDistance = currentNode.distance + GetDistance(currentNode.position, neighborPos);

                // 새로운 경로가 더 짧다면 업데이트
                if (newDistance < neighbor.distance)
                {
                    neighbor.distance = newDistance;
                    neighbor.heuristic = CalculateHeuristic(neighborPos, end);
                    neighbor.previous = currentNode;
                }
            }
        }

        // 최종 경로 생성해서 반환
        return BuildPath(end);
    }

    /// <summary>
    /// JPS 알고리즘을 이용한 길찾기
    /// </summary>
    List<Vector2Int> FindPathJPS(Vector2Int start, Vector2Int end)
    {
        // 시작노드의 휴리스틱 값 설정
        nodes[start].heuristic = CalculateHeuristic(start, end);

        while (openSet.Count > 0)
        {
            Node currentNode = GetClosestUnvisitedNode();
            nodesVisited++;

            if (currentNode == null || currentNode.position == end)
            {
                break;
            }

            closedSet.Add(currentNode);
            openSet.Remove(currentNode);

            // 각 방향에 대해 점프 포인트 탐색
            foreach (Vector2Int direction in GetPrimaryDirections())
            {
                Vector2Int jumpPoint = Jump(currentNode.position, direction, end);
                if (jumpPoint != Vector2Int.zero)
                {
                    if (!nodes.ContainsKey(jumpPoint) || closedSet.Contains(nodes[jumpPoint]))
                        continue;

                    float newDistance = currentNode.distance + 
                        GetDistance(currentNode.position, jumpPoint);

                    Node jumpNode = nodes[jumpPoint];
                    if (newDistance < jumpNode.distance)
                    {
                        jumpNode.distance = newDistance;
                        jumpNode.heuristic = CalculateHeuristic(jumpPoint, end);
                        jumpNode.previous = currentNode;
                        openSet.Add(jumpNode);
                    }
                }
            }
        }

        return BuildPath(end);
    }

    /// <summary>
    /// 두 위치 사이의 이동이 유효한지 확인
    /// </summary>
    private bool IsValidMove(Vector2Int from, Vector2Int to)
    {
        Vector2Int diff = to - from;
        
        // 대각선 이동인 경우
        if (Mathf.Abs(diff.x) == 1 && Mathf.Abs(diff.y) == 1)
        {
            // 대각선으로 이동할 때 양쪽 모서리에 벽이 있는지 확인
            Vector2Int horizontal = new Vector2Int(diff.x, 0);
            Vector2Int vertical = new Vector2Int(0, diff.y);
            
            // 수평 또는 수직 방향 둘다 벽이면 이동 불가
            if (IsWall(from + horizontal) && IsWall(from + vertical))
                return false;
        }
        
        return true;
    }

    /// <summary>
    /// 주어진 방향으로 점프 포인트를 찾음
    /// </summary>
    private Vector2Int Jump(Vector2Int current, Vector2Int direction, Vector2Int end)
    {
        Vector2Int next = current + direction;

        // 맵 밖이거나 벽인 경우
        if (!nodes.ContainsKey(next) || nodes[next].isWall || !IsValidMove(current, next))
            return Vector2Int.zero;

        // 목적지에 도달한 경우
        if (next == end)
            return next;

        // 대각선 이동의 경우
        if (direction.x != 0 && direction.y != 0)
        {
            // 수평, 수직 방향의 강제 이웃 확인
            bool hasHorizontalForced = HasForcedNeighbour(next, new Vector2Int(direction.x, 0));
            bool hasVerticalForced = HasForcedNeighbour(next, new Vector2Int(0, direction.y));
            
            // 수평 또는 수직 방향으로 강제 이웃이 있다면 현재 위치가 점프 포인트
            if (hasHorizontalForced || hasVerticalForced)
                return next;

            // 수평, 수직 방향으로 재귀적으로 점프
            if (Jump(next, new Vector2Int(direction.x, 0), end) != Vector2Int.zero ||
                Jump(next, new Vector2Int(0, direction.y), end) != Vector2Int.zero)
            {
                return next;
            }
        }
        // 수직/수평 이동의 경우
        else
        {
            // 강제 이웃 확인
            if (HasForcedNeighbour(next, direction))
                return next;
        }

        // 점프 포인트를 찾지 못했다면 계속 진행
        return Jump(next, direction, end);
    }
    

    /// <summary>
    /// 강제 이웃이 있는지 확인
    /// </summary>
    private bool HasForcedNeighbour(Vector2Int pos, Vector2Int direction)
    {
        // 수평 이동시
        if (direction.x != 0)
        {
            // 위/아래 벽이 있고 대각선 방향이 열려있는지 확인
            return (IsWall(pos + Vector2Int.up) && !IsWall(pos + direction + Vector2Int.up)) ||
                (IsWall(pos + Vector2Int.down) && !IsWall(pos + direction + Vector2Int.down));
        }
        // 수직 이동시
        else if (direction.y != 0)
        {
            // 좌/우 벽이 있고 대각선 방향이 열려있는지 확인
            return (IsWall(pos + Vector2Int.left) && !IsWall(pos + direction + Vector2Int.left)) ||
                (IsWall(pos + Vector2Int.right) && !IsWall(pos + direction + Vector2Int.right));
        }

        return false;
    }

    /// <summary>
    /// 해당 위치가 벽인지 확인
    /// </summary>
    private bool IsWall(Vector2Int pos)
    {
        return !nodes.ContainsKey(pos) || nodes[pos].isWall;
    }

    /// <summary>
    /// 모든 가능한 이동 방향 반환
    /// </summary>
    private IEnumerable<Vector2Int> GetPrimaryDirections()
    {
        // 기본 방향들(상하좌우) 반환
        yield return Vector2Int.right;
        yield return Vector2Int.left;
        yield return Vector2Int.up;
        yield return Vector2Int.down;

        // 대각선 이동이 허용된 경우 대각선 방향도 반환
        if (allowDiagonal)
        {
            foreach (Vector2Int dir in diagonalDirections)
            {
                yield return dir;
            }
        }
    }

    /// <summary>
    /// 도착점에서 시작점까지 역추적해서 경로 생성
    /// </summary>
    /// <param name="end"></param>
    List<Vector2Int> BuildPath(Vector2Int end)
    {
        // 도착점에서 시작점까지 역추적해서 경로 생성
        List<Vector2Int> path = new List<Vector2Int>();

        // 끝점에서부터 시작
        Node currentNode = nodes[end];

        // 현재 노드가 존재 할때까지
        while (currentNode != null)
        {
            // 현재 노드의 포지션을 path에 추가
            path.Add(currentNode.position);

            // 이전 노드를 현재노드로 변경하고 반복
            currentNode = currentNode.previous;
        }

        // 경로를 뒤집어서 시작점부터 도착점까지 순서로 반환
        path.Reverse();
        return path;
    }

    public void ClearPath()
    {
        VisualizePath(null);

        if (currentPath != null)
            currentPath.Clear();

        currentPath = null;

        pathLine.positionCount = 0;
    }

    /// <summary>
    /// 경로 라인 그리기
    /// </summary>
    private void DrawPathLine(List<Vector2Int> path)
    {
        if (path == null || path.Count == 0)
        {
            pathLine.positionCount = 0;
            return;
        }

        // 라인 포인트 개수 설정
        pathLine.positionCount = path.Count;

        // 각 경로 지점을 월드 좌표로 변환하여 라인 포인트 설정
        for (int i = 0; i < path.Count; i++)
        {
            Vector3 worldPos = tileMap.GetCellCenterWorld(new Vector3Int(path[i].x, path[i].y, 0));
            pathLine.SetPosition(i, worldPos);
        }
    }

    /// <summary>
    /// 경로 시각화 메서드
    /// </summary>
    /// <param name="newPath"></param>
    void VisualizePath(List<Vector2Int> newPath)
    {
        if (currentPath != null)
        {
            foreach (Vector2Int pos in currentPath)
            {
                tileMap.SetTile(new Vector3Int(pos.x, pos.y, 0), floorTile);
            }
        }

        if (newPath != null)
        {
            foreach (Vector2Int pos in newPath)
            {
                tileMap.SetTile(new Vector3Int(pos.x, pos.y, 0), pathTile);
            }

            DrawPathLine(newPath);
        }
    }
}
