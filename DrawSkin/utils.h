#ifndef UTILS_H
#define UTILS_H

/* ---------------------------------------------------------------------- */
/* Makrodefinicje preprocesora */

/*
 * Makro eprintf(fmt, args...) o zmiennej liczbie argumentów (składnia
 * standardu C99) jest wykorzystywane do wypisywania informacji
 * pomocniczych do debugowania programu (do standardowego strumienia
 * błędów stderr).  Nie wypisuje nic przy kompilacji programu ze
 * zdefiniowaną stałą preprocesora NDEBUG (CPPFLAGS += -DNDEBUG)
 */
#ifndef NDEBUG
#define eprintf(...) fprintf(stderr, __VA_ARGS__)
#else
#define eprintf(...) ((void)0)
#endif

/*
 * Makro ARRAY_SIZE(arr) zwraca liczbę elementów w tablicy arr.
 * Wyrażenie to jest stałą czasu kompilacji, zatem może na przykład
 * być używane przy definiowaniu nowych tablic.
 *
 * UWAGA: tą definicję preprocesora należy używać tylko na tablicach!
 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))
#endif

// patrz także http://stackoverflow.com/a/14568783/46058
#ifndef hypot3f
#define hypot3f(x,y,z) hypotf(hypot((x),(y)),(z))
#endif

#ifndef fmin3f
#define fmin3f(a,b,c) fminf((a),fminf((b),(c)))
#define fmax3f(a,b,c) fmaxf((a),fmaxf((b),(c)))
#endif

#endif /* !defined(UTILS_H) */
