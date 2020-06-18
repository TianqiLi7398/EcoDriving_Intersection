# Dynamic Programming

<<<<<<< HEAD
We have 3 paramenters here, 
$$
\Delta x, \Delta v, \Delta T
$$ 
## Determin T
1. What we need first is 
$$
a = \frac{\Delta v}{\Delta T}
$$
also we know 
$$
a_{min} \leq a \leq a_{max}  (1)\\
|\Delta T| \geq |\frac{\Delta v}{a_{min}}| \text{ and} \Delta T \geq \frac{\Delta v}{a_{max}}
$$
thus we know the minimal time is
$$
\Delta T \geq \frac{\Delta v}{min(|a_{min}, |a_{max}|)} (2)
$$
2. The upper bound of T
$$
\Delta T \leq \frac{\Delta x}{v_{max}} (3)
$$
so plug 
$$
\Delta x = 10, a_{min}, a_{max} = -5, 8, v_{max} = 22
$$
by 3 we get 

$$
\Delta T \leq \frac{10}{22} \approx 0.4545
$$

then from (2) we get

$$
\frac{\Delta v}{5} \leq \Delta T \leq \frac{10}{22} \approx 0.4545 
$$

Thus here $$\Delta v$$ could be 2 or 1.

# Forget everything above, this won't work at all!!!

But at least we should know that from one node to another node, the time is less than the minimal time (3)
=======
We have 3 paramenters here, $x$
$$
asdf asdf 
$$
>>>>>>> 9a932c3271f85523aba594241ca3aaf6fdc82bb8
