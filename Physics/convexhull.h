#ifndef _CONVEXHULL
#define _CONVEXHULL

#include <algorithm>
#include <vector>

using std::sort;
using std::vector;

#define sz(X) ((int)((X).size()))

int sign(float x, float eps = FLT_EPSILON)
{
	return (x < -eps ? -1 : x > eps);
}

bool cmp(const b2Vec2 &a, const b2Vec2 &b)
{
	if (sign(a.x - b.x))
		return sign(a.x - b.x) < 0;
	return sign(a.y - b.y) < 0;
}

float det(const b2Vec2 &a, const b2Vec2 &b)
{
	return a.x * b.y - a.y * b.x;
}

bool equal_cmp(const b2Vec2 &a, const b2Vec2 &b)
{
	if (!sign(a.x - b.x) && !sign(a.y - b.y))
		return 1;
	return 0;
}

vector <b2Vec2> convexhull(vector <b2Vec2> a)
{
	vector <b2Vec2> b;
	b.resize(sz(a) + 1);
	int m = 0;
	sort(a.begin(), a.end(), cmp);
	int n = 0;
	for (int i = 1; i < sz(a); i ++)
		if (!equal_cmp(a[i], a[n]))
			a[++ n] = a[i];
	a.resize(n);
	for (int i = 0; i < sz(a); i ++)
	{
		while (m >= 2 && sign(det(b[m] - b[m - 1], a[i] - b[m])) <= 0)
			m --;
		b[++ m] = a[i];
	}
	int rev = m;
	for (int i = sz(a) - 2; i >= 0; i --)
	{
		while (m > rev && sign(det(b[m] - b[m - 1], a[i] - b[m])) <= 0)
			m --;
		b[++ m] = a[i];
	}
	m --;
	for (int i = 0; i < m; i ++)
		b[i] = b[i + 1];
	b.resize(m);
	return b;
}

#endif
