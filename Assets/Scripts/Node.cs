using UnityEngine;

/// <summary>
/// 경로탐색에 사용되는 노드 클래스
/// </summary>
public class Node
{
    public Vector2Int position;                 // 노드의 그리드 위치
    public float distance = Mathf.Infinity;     // Dijkstrat 에서는 시작 위치에서부터의 거리 (AStar에서는 G값)
    public int heuristic = 0;                   // 종료점까지의 추정 거리 H값
    public int weight = 0;
    public Node previous = null;                // 최단 경로에서 이전의 노드
    public bool isWall = false;                 // 벽 여부

    public Node(Vector2Int position)
    {
        this.position = position;
    }
}