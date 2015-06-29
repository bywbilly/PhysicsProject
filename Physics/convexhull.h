#ifndef CONVEXHULL
#define CONVEXHULL

#include <algorithm>
#include <vector>

using std::sort;
using std::vector;

#define sz(X) ((int)((X).size()))

int sign(double x, double eps = 1e-8)
{
	return (x < -eps ? -1 : x > eps);
}

bool cmp(const b2vec2 &a, const b2vec2 &b)
{
	if (sign(a.x - b.x))
		return sign(a.x - b.x) < 0;
	return sign(a.y - b.y) < 0;
}

double det(const b2vec2 &a, const b2vec2 &b)
{
	return a.x * b.y - a.y * b.x;
}

b2vec2 operator - (const b2vec2 &a, const b2vec2 &b)
{
	return b2vec2(a.x - b.x, a.y - b.y);
}

vector <b2vec2> convexhull(vector <b2vec2> a)
{
	vector <b2vec2> b;
	b.resize(sz(a));
	int m = 0;
	sort(a.begin(), a.end(), cmp);
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
