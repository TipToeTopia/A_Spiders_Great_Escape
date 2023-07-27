#pragma once

struct CONNECTION // connections for nodes
{
	int nEnd;		// index of the destination node
	float cost;		// cost of the transition
};

struct NODE // nodes themselves
{
	CVector pos;				//  position of node
	list<CONNECTION> conlist;	// list of connections made

	float costSoFar; // A* values
	int nConnection;
	bool open, closed;
};

class CMyGame : public CGame
{
	CSprite m_player;				// Spider (player)
	CSpriteList m_tiles;			// Tiles
	CSpriteList m_nodes;			// Nodes
	static char* m_tileLayout[12];	// Tiles layout

	vector<NODE> m_graph;           // graph node
	list<CVector> m_waypoints;      // waypoints

	CSpriteVector m_guards;			// Enemies
	CSprite* m_sight;	            // Positive sight test!

	CSprite* guard_movement;        // guard patrol

	CSprite* m_hearing;             // guard hearing/ sensing radius

	CSprite* m_intersection;        // guards after seeing enemy will chase, regardless of intersection

	CSoundPlayer playerwalking;     // player movement sound (implemented to justify the guard chasing player if player within hearing distance of guard)

public:
	CMyGame(void);
	~CMyGame(void);

	// Per-Frame Callback Funtions (must be implemented!)
	virtual void OnUpdate();
	virtual void OnDraw(CGraphics* g);

	// Game Life Cycle
	virtual void OnInitialize();
	virtual void OnDisplayMenu();
	virtual void OnStartGame();
	virtual void OnStartLevel(Sint16 nLevel);
	virtual void OnGameOver();
	virtual void OnTerminate();

	// Keyboard Event Handlers
	virtual void OnKeyDown(SDLKey sym, SDLMod mod, Uint16 unicode);
	virtual void OnKeyUp(SDLKey sym, SDLMod mod, Uint16 unicode);

	// Mouse Events Handlers
	virtual void OnMouseMove(Uint16 x, Uint16 y, Sint16 relx, Sint16 rely, bool bLeft, bool bRight, bool bMiddle);
	virtual void OnLButtonDown(Uint16 x, Uint16 y);
	virtual void OnLButtonUp(Uint16 x, Uint16 y);
	virtual void OnRButtonDown(Uint16 x, Uint16 y);
	virtual void OnRButtonUp(Uint16 x, Uint16 y);
	virtual void OnMButtonDown(Uint16 x, Uint16 y);
	virtual void OnMButtonUp(Uint16 x, Uint16 y);
};

