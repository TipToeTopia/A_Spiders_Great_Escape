#include "stdafx.h"
#include "MyGame.h"

#pragma warning (disable: 4244)

int speed = 110; // patrol route speed
int phase; // phase patrol routes
int phase2; // phase patrol routes (guard 2)

char* CMyGame::m_tileLayout[12] = // tile layout
{
	"XXXXXXXXXXXXXXXXXXXX",
	"X           X  XXX X",
	"X  XXX  XXXXX  XXX X",
	"X  XXX         XXX X",
	"X  XXX  XXXXX      X",
	"   XXX  XXXXX  XXXXX",
	"        XX         X",
	"X  XXX         X   X",
	"X  XXX  XX     X   X",
	"X  XXX  XX XXXXX   X",
	"X                  X",
	"XXXXXXXXXXXXXXXXXXXX",
};

float Coords[][2] = // coordinates (nodes)
{ 
	{96,96},{125,96},{154,96},{183,96},{212,96},{241,96},
    {270,96},{299,96},{328,96},{357,96},{386,96},{415,96}, 
	{444,96},{473,96},{502,96},{531,96},{560,96},{589,96},
	{618,96},{647,96},{676,96},{705,96},{734,96},{96,96},
	{96,125},{96,154},{96,183},{96,212},{96,241},{96,270},
	{96,299},{96,328},{96,357},{96,386},{96,415},{96,444},
	{96,473},{96,502},{96,531},{96,560},{96,589},{96,618},
	{96,647},{96,676},{415,125},{444,125},{473,125},{415,96},
	{415,125},{415,154},{415,183},{415,212},{415,241},
	{415,270},{415,299},{415,328},{415,357},{415,386}, 
	{415,415},{415,444},{415,473},{415,502},{415,531},
	{415,560},{415,589},{415,618},{415,647},{415,676},
    {444,125},{444,154},{444,183},{444,212},{444,241},
	{444,270},{444,299},{444,328},{444,357},{444,386},
	{444,415},{444,444},{444,473},{444,502},{444,531}, 
	{444,560},{444,589},{444,618},{444,647},{444,676},
	{473,125},{473,154},{473,183},{473,212},{473,241},
	{473,270},{473,299},{473,328},{473,357},{473,386},
	{473,415},{473,444},{473,473},{473,502},{473,531},
	{473,560},{473,589},{473,618},{473,647},{473,676},
	{125,125},{125,154},{125,183},{125,212},{125,241},
	{125,270},{125,299},{125,328},{125,357},{125,386},
	{125,415},{125,444},{125,473},{125,502},{125,531},
	{125,560},{125,589},{125,618},{125,647},{125,676},
	{154,125},{154,154},{154,183},{154,212},{154,241},
	{154,270},{154,299},{154,328},{154,357},{154,386},
	{154,415},{154,444},{154,473},{154,502},{154,531},
	{154,560},{154,589},{154,618},{154,647},{154,676},
	{183,415},{212,415},{241,415},{270,415},{299,415},
	{328,415},{357,415},{386,415},{183,676},{212,676},
	{241,676},{270,676},{299,676},{328,676},{357,676},
	{386,676},{502,225.5},{531,225.5},{560,225.5},
	{589,225.5},{618,225.5},{647,225.5},{676,225.5},{705,225.5},
	{734,225.5},{763,225.5},{792,225.5},{821,225.5},{850,225.5},
	{879,225.5},{908,225.5},{879,96},{879,125},{879,154},{879,183},
	{879,96},{879,270},{879,299},{879,328},{879,357},{908,96},
	{908,125},{908,154},{908,183},{908,270},{908,299},
	{908,328},{908,357},{937,284.5},{966,284.5},{995,284.5},
	{1024,284.5},{1053,284.5},{1082,284.5},{1111,284.5},{1140,284.5},
	{1169,284.5},{1183.5,255.5},{1183.5,226.5},{1183.5,197.5},
	{1183.5,168.5},{1183.5,139.5},{1183.5,110.5},{1183.5,81.5},
	{502,473},{531,473},{560,473},{589,473},{618,473},{647,473},
	{676,473},{502,676},{531,676},{560,676},{589,676},{618,676},
	{647,676},{676,676},{676,502},{676,531},{676,560},{676,589}, 
	{676,618},{676,647},{676,444},{676,415},{705,415},{734,415},
	{763,415},{792,415},{821,415},{850,415},{879,415},
	{908,415},{937,415},{966,415},{995,415},{1024,415},{1053,415},
	{1082,415},{1111,415},{1140,415},{1169,415},{908,386},{879,386},
	{705,676},{734,676},{763,676},{792,676},{821,676},{850,676},
	{879,676},{908,676},{937,676},{966,676},{995,676},{1024,676},
	{1053,676},{1082,676},{1111,676},{1140,676},{1169,676},{705,502},{705,531},
	{705,473},{705,444},{734,502},{734,531},{734,473},{734,444},{763,502},
	{763,531},{763,473},{763,444},{792,502},{792,531},{792,473},
	{792,444},{821,502},{821,531},{821,473},{821,444},{850,502},
	{850,531},{850,473},{850,444},{879,502},{879,531},{879,473},
	{879,444},{908,502},{908,531},{908,473},{908,444},{1169,647},
	{1169,618},{1169,589},{1169,560},{1169,531},{1169,502},
	{1169,473},{1169,444},{1140,647},{1140,618},{1140,589},
	{1140,560},{1140,531},{1140,502},{1140,473},{1140,444},{1111,647},
	{1111,618},{1111,589},{1111,560},{1111,531},
	{1111,502},{1111,473},{1111,444},{1082,647},{1082,618},{1082,589},
	{1082,560},{1082,531},{1082,502},{1082,473},{1082,444},
    {62,357},{62,386},{62,415},{33,357},{33,386},{33,415}
};

int Connections[][2] = // links between nodes (waypoints)
{
	{0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,8},{8,9},{9,10},

	{10,11},{11,12},{12,13},{13,14},{14,15},{15,16},{16,17},{17,18},{18,19},{19,20},{20,21},{21,22},{0,24},
	{24,25},{25,26},{26,27},{27,28},{28,29},{29,30},{30,31},{31,32},{32,33},{33,34},{34,35},{35,36},{36,37},
	{37,38},{38,39},{39,40},{40,41},{41,42},{42,43},{108,1},{108,24},{108,128},{128,2},{23,108},{24,1},
	{128,1},{128,3},{109,108},{129,128},{25,109},{109,129},{109,110},{110,111},{111,112},{112,113},{113,114},
	{114,115},{115,116},{116,117},{117,118},{118,119},{119,120},{120,121},{121,122},{122,123},{123,124},
	{124,125},{125,126},{126,127},{129,130},{130,131},{131,132},{132,133},{133,134},{134,135},{135,136},
	{136,137},{137,138},{138,139},{139,140},{140,141},{141,142},{142,143},{143,144},{144,145},{145,146},
	{146,147},{26,110},{110,130},{27,111},{111,131},{28,112},{112,132},{29,113},{113,133},{30,114},{114,134},
	{31,115},{115,135},{32,116},{116,136},{33,117},{117,137},{34,118},{118,138},{35,119},{119,139},{36,120},
	{120,140},{37,121},{121,141},{38,122},{122,142},{39,123},{123,143},{40,124},{124,144},{41,125},{125,145},
	{42,126},{126,146},{43,127},{127,147},{138,148},{148,149},{149,150},{150,151},{147,156},{156,157},
	{157,158},{158,159},{159,160},{160,161},{161,162},{162,163},{151,152},{152,153},{153,154},{154,155},
	{139,148},{137,148},{146,156},{47,48},{48,49},{49,50},{50,51},{51,52},{52,53},{53,54},{54,55},{55,56},
	{56,57},{57,58},{58,59},{59,60},{60,61},{61,62},{62,63},{63,64},{64,65},{65,66},{66,67},{163,66},{10,48},
	{155,58},{155,57},{155,59},{163,67},{12,68},{68,69},{69,70},{70,71},{71,72},{72,73},{73,74},{74,75},
	{75,76},{76,77},{77,78},{78,79},{70,80},{80,81},{81,82},{82,83},{83,84},{84,85},{85,86},{86,87},{48,68},
	{49,69},{50,70},{51,71},{52,72},{53,73},{54,74},{55,75},{56,76},{57,77},{58,78},{59,79},{60,80},{61,81},
	{62,82},{63,83},{64,84},{65,85},{66,86},{67,87},{13,88},{88,89},{89,90},{90,91},{91,92},{92,93},{93,94},
	{94,95},{95,96},{96,97},{97,98},{98,99},{99,100},{100,101},{68,88},{69,89},{70,90},{71,91},{72,92},
	{73,93},{74,94},{75,95},{76,96},{77,97},{78,98},{79,99},{80,100},{81,101},{82,102},{83,103},{84,104},
	{85,105},{86,106},{87,107},{88,14},{92,164},{91,164},{99,212},{108,2},{31,334},{32,334},{334,337},
	{337,334},{337,338},{338,339},{334,335},{335,336},{334,335},{335,336},{338,335},{335,33},{339,336},
	{336,34},{336,35},

	{169,170},{170,171},{171,172},{172,173},{173,174},{174,175},{175,176},{176,177},{177,178},{177,182},
	{178,191},{176,182},{182,181},{181,180},{180,183},{178,191},{191,190},{190,189},{189,188},{177,184},
	{178,192},{192,196},{184,176},{196,197},{197,198},{198,199},{199,200},{200,201},{201,202},{202,203},
	{203,204},{204,205},{205,206},{206,207},{207,208},{208,209},{209,210},{210,211},{184,192},{182,191},
	{181,190},{180,189},{183,188},{196,193},{192,193},{184,185},{185,193},{186,187},{194,195},{186,194},
	{187,195},{185,186},{194,193},{107,219},{219,220},{220,221},{221,222},{222,223},{223,224},{268,269},
	{224,225},{225,231},{231,230},{230,229},{229,228},{228,227},{227,226},{226,218},{218,232},{232,233},
	{218,217},{217,216},{216,215},{215,214},{214,213},{213,212},{212,100},{271,227},{270,226},{272,218},
	{273,232},{234,233},{271,270},{270,272},{272,273},{273,234},{234,235},{235,236},{236,237},{237,238},
	{238,239},{239,240},{240,241},{241,242},{235,277},{277,276},{276,274},{274,275},{236,281},{281,280},
	{280,278},{278,279},{285,237},{285,284},{284,282},{282,283},{238,289},{289,288},{288,286},{286,287},
	{239,293},{293,292},{292,290},{290,291},{240,297},{297,296},{296,294},{294,295},{241,301},{301,300},
	{300,298},{298,299},{271,275},{275,279},{279,283},{283,287},{287,291},{291,295},{295,299},{270,274},
	{274,278},{278,282},{282,286},{286,290},{290,294},{294,298},{272,276},{276,280},{280,284},{284,288},
	{288,292},{292,296},{296,300},{273,277},{277,281},{281,285},{285,289},{289,293},{293,297},{297,301},
	{187,252},{195,251},{252,240},{251,241},{242,243},{243,244},{244,245},{245,246},{246,247},{247,248},
	{248,249},{249,250},{247,333},{333,332},{332,331},{331,330},{330,329},{329,328},{328,327},{327,326},
	{248,325},{325,324},{324,323},{323,322},{322,321},{321,320},{320,319},{319,318},{318,267},{249,317},
	{317,316},{316,315},{315,314},{314,313},{313,312},{312,311},{311,310},{310,268},{250,309},{309,308},
	{308,307},{307,306},{306,305},{305,304},{304,303},{303,302},{302,269},{326,266},{333,325},{325,317},
	{317,309},{332,324},{324,316},{316,308},{331,323},{323,315},{315,307},{330,322},{322,314},{314,306},
	{329,321},{321,313},{313,305},{328,320},{320,312},{312,304},{327,319},{319,311},{311,303},{326,318},
	{318,310},{310,302},{225,253},{253,254},{254,255},{255,256},{256,257},{257,258},{258,259},{259,260},
	{260,261},{261,262},{262,263},{263,264},{264,265},{265,266},{266,267},{267,268},{164,165},{165,166},
	{166,167},{167,168},{168,169},{101,102},{102,103},{103,104},{104,105},{105,106},{106,107},{106,219},
	{101,212},{231,224},{253,231},{228,271},{232,217},{217,226},{301,242},{265,326},{333,246},{252,239},
	{242,251},{251,252}
};

bool PathFind(vector<NODE>& graph, int nStart, int nGoal, vector<int>& path) // Dijkstra algorithm implementation
{
	list<int> open; // all lists

	// mark all nodes in the graph as unvisited
	for (unsigned i = 0; i < graph.size(); i++)
		graph[i].open = false;

	// open the Start node
	graph[nStart].costSoFar = 0;
	graph[nStart].nConnection = -1;
	graph[nStart].open = true;
	open.push_back(nStart);

	while (open.size() > 0)
	{
		// Find the element with the smallest costSoFar
		// iMin is the iterator (pointer) to its position in the opn list
		list<int>::iterator iCurrent = min_element(open.begin(), open.end(), [graph](int i, int j) -> bool {
			return graph[i].costSoFar < graph[j].costSoFar;
			});
		int curNode = *iCurrent;
		float coastSoFar = graph[curNode].costSoFar;

		// If the end node found, then end
		if (curNode == nGoal)
			break;

		// Else, visit all the connections
		for (CONNECTION conn : graph[curNode].conlist)
		{
			int endNode = conn.nEnd;
			float newCostSoFar = coastSoFar + conn.cost;

			// for open nodes, ignore if the current route worse then the route already found
			if (graph[endNode].open && graph[endNode].costSoFar <= newCostSoFar)
				continue;

			// Closer route
			graph[endNode].costSoFar = newCostSoFar;
			graph[endNode].nConnection = curNode;

			// if unvisited yet, add to the open list
			if (!graph[endNode].open)
			{
				graph[endNode].open = true;
				open.push_back(endNode);
			}
		}

		// Now we can close the current graph
		graph[curNode].closed = true;
		open.erase(iCurrent);
	}

	// Collect the path from the generated graph data
	if (open.size() == 0)
	{
		return false; // path not found
	}

	int i = nGoal;
	while (graph[i].nConnection >= 0)
	{
		path.push_back(i);
		i = graph[i].nConnection;
	}
	path.push_back(i);

	reverse(path.begin(), path.end());
	return true;
}

CMyGame::CMyGame(void) :
	m_player(544, 96, 64, 64, 0)
{
	m_sight = NULL; // set to null
	m_hearing = NULL;
	
	m_player.LoadAnimation("Spider64.png", "walk", CSprite::Sheet(4, 2).Col(0).From(0).To(1)); // animations
	m_player.LoadAnimation("Spider64.png", "idle", CSprite::Sheet(4, 2).Col(2).From(0).To(1));
	m_player.SetAnimation("idle", 4);

	// graph structure nodes
	for (float* coord : Coords)
		m_graph.push_back(NODE{ CVector(coord[0], coord[1]) });

	// graph structure connections
	for (int* conn : Connections)
	{
		int ind1 = conn[0];
		int ind2 = conn[1];
		NODE& node1 = m_graph[ind1];
		NODE& node2 = m_graph[ind2];
		float dist = Distance(node1.pos, node2.pos);

		node1.conlist.push_back(CONNECTION{ ind2, dist });
		node2.conlist.push_back(CONNECTION{ ind1, dist });
	}
}

bool Intersection(CVector a, CVector b, CVector c, CVector d, float& k1, float& k2) // info for intersection needed (tests for intersection)
{
	CVector v1 = b - a;
	CVector v2 = d - c;
	CVector con = c - a;

	// if lines are parrelel or perpendicular
	float det = v1.m_x * v2.m_y - v1.m_y * v2.m_x;

	if (det != 0)
	{
		// point from 0 to 1 where interdection is hit
		k1 = (v2.m_y * con.m_x - v2.m_x * con.m_y) / det;
		k2 = (v1.m_y * con.m_x - v1.m_x * con.m_y) / det;

		return true;
	}
	else
	{
		return false;
	}
}

bool Intersection(CVector a, CVector b, CVector c, CVector d) // info for intersection needed (simplified version)
{
	float k1, k2;
	if (!Intersection(a, b, c, d, k1, k2))
		return false;
	return k1 >= 0 && k1 <= 1.f && k2 >= 0 && k2 <= 1.f;
}

CMyGame::~CMyGame(void)
{
}

void CMyGame::OnUpdate() // per frame
{
	if (!IsGameMode()) return;

	Uint32 t = GetTime();

	// spider: follow the waypoints
	if (!m_waypoints.empty())
	{
		// If spider not moving, start moving to the first waypoint
		if (m_player.GetSpeed() < 1)
		{
			if (m_sight == NULL || m_hearing == NULL) // player isn't seen by guards
			{
				m_player.SetSpeed(220); // undetected speed
			}
			else
			{
				m_player.SetSpeed(250); // detected speed
			}
			m_player.SetDirection(m_waypoints.front() - m_player.GetPosition());
			playerwalking.Play("Person running.wav"); // movement sound
		}

		// Passed the waypoint?
		CVector v = m_waypoints.front() - m_player.GetPosition();
		if (Dot(m_player.GetVelocity(), v) < 0) // if so movement should terminate
		{
			// Stop movement
			m_waypoints.pop_front();
			if (m_waypoints.empty())
			m_player.SetAnimation("idle");
			m_player.SetVelocity(0, 0);
			m_player.SetRotation(0);
		}
	}
	m_player.Update(t);

	for (CSprite* pGuard : m_guards)
	{
		pGuard->Update(GetTime());
	}

	// CONE OF VISION TEST AND PATROL ROUTES
	for (CSprite* pGuard : m_guards)
	{
		if (pGuard->HitTest(&m_player)) // if guard catches player, game over :( 
		{
			GameOver();
		}

		// by default, we assume each guard can become attacker through seeing and hearing
		m_sight = pGuard;
		m_hearing = pGuard;
		guard_movement = pGuard;
		m_intersection = pGuard; // intersection and guard movement are to maintain certain scenarios

		if (guard_movement) // patrol state + routes
		{
			if (phase == 1 && m_guards[0]->GetPosition().m_x < 96) // B to C (Guard 0)
			{
				phase = 2;
				m_guards[0]->SetVelocity(0, speed);
				m_guards[0]->SetAnimation("walkU");
			}
			if (phase == 2 && m_guards[0]->GetPosition().m_y > 680) // random patrol instance (guard may go from C to D or instead from C back to B)
			{
				int r = rand() % 2;
				if (r == 0) // if this is called guard patrols from C back to B
				{
					phase = 5;
					m_guards[0]->SetVelocity(0, -speed);
					m_guards[0]->SetAnimation("walkD");
				}
				if (r == 1) // guard patrols from C to D
				{
					phase = 3;
					m_guards[0]->SetVelocity(speed, 0);
					m_guards[0]->SetAnimation("walkR");
				}
			}
			if (phase == 3 && m_guards[0]->GetPosition().m_x > 416) // movement
			{
				phase = 4;
				m_guards[0]->SetVelocity(0, -speed);
				m_guards[0]->SetAnimation("walkD");
			}
			if (phase == 4 && m_guards[0]->GetPosition().m_y < 95) // movement
			{
				int r = rand() % 2;
				if (r == 0) // Other randomised routes
				{
					phase = 7;
					m_guards[0]->SetVelocity(0, speed);
					m_guards[0]->SetAnimation("walkU");
				}
				if (r == 1) // guard patrols from C to D
				{
					phase = 1;
					m_guards[0]->SetVelocity(-speed, 0);
					m_guards[0]->SetAnimation("walkL");
				}
			}
			if (phase == 5 && m_guards[0]->GetPosition().m_y < 95) // movement if C back to B is called
			{
				phase = 6;
				m_guards[0]->SetVelocity(speed, 0);
				m_guards[0]->SetAnimation("walkR");
			}
			if (phase == 6 && m_guards[0]->GetPosition().m_x > 416) // movement if C back to B is called
			{
				int r = rand() % 2;
				if (r == 0) // Other randomised routes
				{
					phase = 7;
					m_guards[0]->SetVelocity(0, speed);
					m_guards[0]->SetAnimation("walkU");
				}
				if (r == 1) // guard patrols from C to D
				{
					phase = 1;
					m_guards[0]->SetVelocity(-speed, 0);
					m_guards[0]->SetAnimation("walkL");
				}
			}
			if (phase == 7 && m_guards[0]->GetPosition().m_y > 680) // movement if C back to B is called (we reset back to 1 and phase 2 is instantly called; resetting the process)
			{
				phase = 1;
				m_guards[0]->SetVelocity(-speed, 0);
				m_guards[0]->SetAnimation("walkL");
			}
			if (m_guards[1]->GetPosition().m_y > 690) // Guard 1 is predictabe to have variation for the player to consider (simply goes from point A to B and back), contextually guarding a corridor is ideal for the guards
			{
				m_guards[1]->SetAnimation("walkD");
				m_guards[1]->SetVelocity(0, -speed);
			}
			if (m_guards[1]->GetPosition().m_y < 90)
			{
				m_guards[1]->SetAnimation("walkU");
				m_guards[1]->SetVelocity(0, speed);
			}
			if (phase2 == 1 && m_guards[2]->GetPosition().m_x < 676) // B to C (Guard 2)
			{
				phase2 = 2;
				m_guards[2]->SetVelocity(0, speed);
				m_guards[2]->SetAnimation("walkU");
			}
			if (phase2 == 2 && m_guards[2]->GetPosition().m_y > 680) // random patrol instance (guard may go from C to D or instead from C back to B)
			{
				int r = rand() % 2;
				if (r == 0) // if this is called guard 2 patrols from C back to B
				{
					phase2 = 5;
					m_guards[2]->SetVelocity(0, -speed);
					m_guards[2]->SetAnimation("walkD");
				}
				if (r == 1) // guard 2 patrols from C to D
				{
					phase2 = 3;
                    m_guards[2]->SetVelocity(speed, 0);
                    m_guards[2]->SetAnimation("walkR");
				}
			}
			if (phase2 == 3 && m_guards[2]->GetPosition().m_x > 1124) // movement
			{
				phase2 = 4;
				m_guards[2]->SetVelocity(0, -speed);
				m_guards[2]->SetAnimation("walkD");

			}
			if (phase2 == 4 && m_guards[2]->GetPosition().m_y < 416) // movement
			{
				int r = rand() % 2;
				if (r == 0) // Other randomised routes
				{
					phase2 = 7;
					m_guards[2]->SetVelocity(0, speed);
					m_guards[2]->SetAnimation("walkU");
				}
				if (r == 1) // guard patrols from C to D
				{
					phase2 = 1;
					m_guards[2]->SetVelocity(-speed, 0);
					m_guards[2]->SetAnimation("walkL");
				}
			}
			if (phase2 == 5 && m_guards[2]->GetPosition().m_y < 416) // movement if C back to B is called
			{
				phase2 = 6;
				m_guards[2]->SetVelocity(speed, 0);
				m_guards[2]->SetAnimation("walkR");
			}
			if (phase2 == 6 && m_guards[2]->GetPosition().m_x > 1124) // movement if C back to B is called
			{
				int r = rand() % 2;
				if (r == 0) // Other randomised routes
				{
					phase2 = 7;
					m_guards[2]->SetVelocity(0, speed);
					m_guards[2]->SetAnimation("walkU");
				}
				if (r == 1)
				{
					phase2 = 1;
					m_guards[2]->SetVelocity(-speed, 0);
					m_guards[2]->SetAnimation("walkL");
				}
			}
			if (phase2 == 7 && m_guards[2]->GetPosition().m_y > 680) // movement if C back to B is called (we reset back to 1 and phase 2 is instantly called; resetting the process)
			{
				phase2 = 1;
				m_guards[2]->SetVelocity(-speed, 0);
				m_guards[2]->SetAnimation("walkL");
			}
		}

		// browse through all tiles - if line of sight test shows any tile to obscure the player, guards can't see player and therefore won't persuit player
		for (CSprite* pTile : m_tiles)
		{
			CVector d(pTile->GetLeft(), pTile->GetTop()); // points for which will make up the 2 diagonal lines
			CVector c(pTile->GetRight(), pTile->GetBottom());
			CVector e(pTile->GetLeft(), pTile->GetBottom());
			CVector f(pTile->GetRight(), pTile->GetTop());

			if (m_intersection) // used so if the guard sees the player it won't go back to its patrol state if theres an obstacle (logically the guard has seen the player and will chase no matter what)
			{
				bool Intersection(CVector m_pkiller, CVector m_player, CVector c, CVector d);
				{
					if (Intersection(pGuard->GetPosition(), m_player.GetPosition(), c, d)) // lines intersect which is a replication of an obstacle in the way of guard and player; therefore guard doesn't engage
					{
						m_sight = NULL; // therefore guard won't chase
						m_hearing = NULL; // therefore guard won't chase
					}
				}
				bool Intersection(CVector m_pkiller, CVector m_player, CVector e, CVector f);
				{
					if (Intersection(pGuard->GetPosition(), m_player.GetPosition(), e, f)) // lines intersect which is a replication of an obstacle in the way of guard and player; therefore guard doesn't engage
					{
						m_sight = NULL; // therefore guard won't chase
						m_hearing = NULL; // therefore guard won't chase
					}
				}
			}
		}

		// if the player is in cone of vision of guard
		if (m_sight)
		{
			CVector v = m_player.GetPosition() - pGuard->GetPosition(); // line 
			if ((Dot(Normalize(v), Normalize(pGuard->GetVelocity())) <= cos(50)) || Distance(pGuard->GetPosition(), m_player.GetPosition()) > 200) // cone of vision limited to visibility distance of 200 and 50 degrees
			{
				m_sight = NULL; // so if these conditions are satisfied the guards can't see player so won't attack
			}
		}

		// if player is inside guards cone of sight and no intersection then he chases the player
		if (m_sight)
		{
			if (Distance(pGuard->GetPosition(), m_player.GetPosition()) > 1000) // extra maintainance for chasing after player is identified
			{
				m_sight = NULL;
            }
			else
			{
				guard_movement = NULL; // movement is changed from patrol state to attack
				m_intersection = NULL; // guard will chase, even if theres an obstacle in the way during the chase phase
				CVector sight = m_player.GetPosition() - pGuard->GetPosition();
				sight.Normalise();
				pGuard->SetDirection(sight);
				pGuard->SetSpeed(200); // guard sees the player so increases his speed in an attempt to catch the player
			}
		}

		// if player is inside guards hearing/ sensing radius then he chases the player
		if (m_hearing)
		{
			if (Distance(pGuard->GetPosition(), m_player.GetPosition()) < 75) // hearing/ sensing radius (they cant hear through walls though as crawling is relatively quiet)
			{
				guard_movement = NULL; // movement is changed from patrol state to attack
				m_intersection = NULL; // guard will chase, even if theres an obstacle in the way during the chase phase
				CVector hear = m_player.GetPosition() - pGuard->GetPosition();
				hear.Normalise();
				pGuard->SetDirection(hear);
				pGuard->SetSpeed(200); // guard sense the player near him and begins to chase
			}
		}
	}

	// WINNING CONDITION
	if (m_player.GetLeft() < 5) // escaped
	{
		GameOver();
	}
}

void CMyGame::OnDraw(CGraphics* g) // graphics
{
	for (NODE n : m_graph)
	{
		for (CONNECTION c : n.conlist)
		{
			g->DrawLine(n.pos, m_graph[c.nEnd].pos, CColor::Black());
		}
	}
	m_tiles.for_each(&CSprite::Draw, g);
	m_player.Draw(g);
	for (CSprite* pGuard : m_guards)
	{
		pGuard->Draw(g);
	}
	if (m_player.GetLeft() < 5)
	{
		*g << font(48) << color(CColor::Red()) << vcenter << center << "YOU ESCAPED!" << endl;
	}
	for (CSprite* pGuard : m_guards)
	{
		if (pGuard->HitTest(&m_player))
		{
			*g << font(48) << color(CColor::Blue()) << vcenter << center << "BUSTED!" << endl;
		}
	}
}

// one time initialisation
void CMyGame::OnInitialize()
{
	// Create Tiles
	for (int y = 0; y < 12; y++)
	{
		for (int x = 0; x < 20; x++)
		{
			if (m_tileLayout[y][x] == ' ')
				continue;

			int nTile = 5;
			if (y > 0 && m_tileLayout[y - 1][x] == ' ') nTile -= 3;
			if (y < 11 && m_tileLayout[y + 1][x] == ' ') nTile += 3;
			if (x > 0 && m_tileLayout[y][x - 1] == ' ') nTile--;
			if (x < 20 && m_tileLayout[y][x + 1] == ' ') nTile++;
			if (nTile == 5 && x > 0 && y > 0 && m_tileLayout[y - 1][x - 1] == ' ') nTile = 14;
			if (nTile == 5 && x < 20 && y > 0 && m_tileLayout[y - 1][x + 1] == ' ') nTile = 13;
			if (nTile == 5 && x > 0 && y < 11 && m_tileLayout[y + 1][x - 1] == ' ') nTile = 11;
			if (nTile == 5 && x < 20 && y < 11 && m_tileLayout[y + 1][x + 1] == ' ') nTile = 10;

			nTile--;
			m_tiles.push_back(new CSprite(x * 64.f + 32.f, y * 64.f + 32.f, new CGraphics("grass.bmp", 3, 5, nTile % 3, nTile / 3), 0));
		}
	}
		
	// Create Nodes
	int i = 0;
	for (NODE n : m_graph)
	{
		stringstream s;
		s << i++;
		m_nodes.push_back(new CSpriteOval(n.pos, 12, CColor::White(), CColor::Black(), 0));
		m_nodes.push_back(new CSpriteText(n.pos, "arial.ttf", 14, s.str(), CColor::Black(), 0));
	}

	// create animations
	for (int i = 0; i < 4; i++)
	{
		CSprite* pGuard = new CSprite(0, 0, "guard.png", 0);
		pGuard->LoadAnimation("guard.png", "walkR", CSprite::Sheet(13, 21).Row(9).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkD", CSprite::Sheet(13, 21).Row(10).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkL", CSprite::Sheet(13, 21).Row(11).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkU", CSprite::Sheet(13, 21).Row(12).From(0).To(8));
		m_guards.push_back(pGuard);
	}
}

void CMyGame::OnDisplayMenu()
{
	StartGame();	// exits the menu mode and starts the game mode
}

// called when a new game is started
// as a second phase after a menu or a welcome screen
void CMyGame::OnStartGame()
{
	m_player.SetPosition(1184, 96);

	m_guards[0]->SetPosition(750, 96);
	m_guards[0]->SetAnimation("walkL");
	m_guards[0]->SetVelocity(CVector(-speed, 0));

	m_guards[1]->SetPosition(476, 160);
	m_guards[1]->SetAnimation("walkU");
	m_guards[1]->SetVelocity(CVector(0, speed));

	m_guards[2]->SetPosition(1184, 416);
	m_guards[2]->SetAnimation("walkL");
	m_guards[2]->SetVelocity(CVector(-speed, 0));

	m_guards[3]->SetPosition(-50, 416); // test guard
	m_guards[3]->SetAnimation("walkL");
	
	m_sight = NULL;
	m_hearing = NULL;
	phase = 1;
	phase2 = 1;
}

void CMyGame::OnStartLevel(Sint16 nLevel)
{
}

// called when the game is over
void CMyGame::OnGameOver()
{
}

// one time termination code
void CMyGame::OnTerminate()
{
}

void CMyGame::OnKeyDown(SDLKey sym, SDLMod mod, Uint16 unicode)
{
}

void CMyGame::OnKeyUp(SDLKey sym, SDLMod mod, Uint16 unicode)
{
}

/////////////////////////////////////////////////////
// Mouse Events Handlers

void CMyGame::OnMouseMove(Uint16 x, Uint16 y, Sint16 relx, Sint16 rely, bool bLeft, bool bRight, bool bMiddle)
{
}

void CMyGame::OnLButtonDown(Uint16 x, Uint16 y)
{
	CVector v(x, y); // destination

	if (Distance(v, m_player.GetPosition()) > 500) // used so the player can't just click at exit at route (too easy)
	{
		return;
	}

	if (m_tileLayout[y / 64][x / 64] != ' ') // check if the move is legal
	{
		return;	// cannot go into a wall
	}

	vector<NODE>::iterator iFirst = min_element(m_graph.begin(), m_graph.end(), [this](NODE& n1, NODE& n2) -> bool // first node closest to the spider
	{
		return Distance(n1.pos, m_player.GetPos()) < Distance(n2.pos, m_player.GetPos());
	});

	vector<NODE>::iterator iLast = min_element(m_graph.begin(), m_graph.end(), [v](NODE& n1, NODE& n2) -> bool 	// find the last node closest to the destination
	{
		return Distance(n1.pos, v) < Distance(n2.pos, v);
	});

	int nFirst = iFirst - m_graph.begin();
	int nLast = iLast - m_graph.begin();

	if (!m_waypoints.empty()) 	// remove the current way points and reset the spider
	{
		m_waypoints.clear();
		m_player.SetVelocity(0, 0);
	}

	// call the path finding algorithm to complete the waypoints
	vector<int> path;
	if (PathFind(m_graph, nFirst, nLast, path))
	{
		for (int i : path)
		m_waypoints.push_back(m_graph[i].pos);
		m_waypoints.push_back(v);
	}
}

void CMyGame::OnLButtonUp(Uint16 x, Uint16 y)
{
}

void CMyGame::OnRButtonDown(Uint16 x, Uint16 y)
{
}

void CMyGame::OnRButtonUp(Uint16 x, Uint16 y)
{
}

void CMyGame::OnMButtonDown(Uint16 x, Uint16 y)
{
}

void CMyGame::OnMButtonUp(Uint16 x, Uint16 y)
{
}

// DONE :)