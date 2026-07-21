import math


class PT2Filter:
  """
  Diskreter PT2-Filter mittels Tustin (bilinear transform)
  Übertragungsfunktion (kontinuierlich):
      H(s) =  (w0^2) / (s^2 + 2*zeta*w0*s + w0^2)
  Abtastzeit dt.
  """
  def __init__(self, w0: float, zeta: float, dt: float):
    """
    w0:   Eigenkreisfrequenz [rad/s], bestimmt u.a. die Anstiegszeit
    zeta: Dämpfungsgrad (zeta=1 => kritisch gedämpft)
    dt:   Abtastzeit [s]
    """
    self.w0   = w0
    self.zeta = zeta
    self.dt   = dt
    self.a1, self.a2, self.b0, self.b1, self.b2 = self._design_pt2(self.w0, self.zeta, self.dt)
    self.y1 = 0.0
    self.y2 = 0.0
    self.u1 = 0.0
    self.u2 = 0.0

  def _design_pt2(self, w0, zeta, dt):
    """
    Erzeuge (a1, a2, b0, b1, b2) aus:
       H(s) = w0^2 / [s^2 + 2*zeta*w0 s + w0^2]
    via Tustin (s = (2/dt)*(1-z^-1)/(1+z^-1)).

    Wir bringen G(z) in die Normalform:
        Y(z)/U(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2).

    => Zeitbereich: y[k] = -a1*y[k-1] - a2*y[k-2] + b0*u[k] + b1*u[k-1] + b2*u[k-2].
    """
        
    Ts = dt
    wd = w0
    alpha = 2.0 / Ts
    b2_ = w0**2
    b1_ = 2.0 * w0**2
    b0_ = w0**2
    A2_f1 = alpha**2
    A1_f1 = -2.0 * alpha**2
    A0_f1 = alpha**2
    factor2 = 2.0*zeta*wd*alpha
    A2_f2 = factor2
    A1_f2 = 0.0
    A0_f2 = -factor2
    A2_f3 = wd**2
    A1_f3 = 2.0*(wd**2)
    A0_f3 = wd**2
    A2 = A2_f1 + A2_f2 + A2_f3
    A1 = A1_f1 + A1_f2 + A1_f3
    A0 = A0_f1 + A0_f2 + A0_f3
    B2 = b2_
    B1 = b1_
    B0 = b0_
    b2d = B2 / A2
    b1d = B1 / A2
    b0d = B0 / A2
    a2d = A0 / A2
    a1d = A1 / A2

    return (a1d, a2d, b0d, b1d, b2d)

  def sync(self, target: float):
    steps = compute_saturation_steps(self.w0, self.zeta, self.dt)
    for i in range(1, steps + 1):
      update(target)

  def compute_saturation_steps(self, w0: float, zeta: float, dt: float) -> int:
    """
    Berechnet eine Abschätzung der Schritte, bis der Filter (95% des Endwerts) erreicht ist.
    
    Wir nutzen hier die Abschätzung:
        T_s = 4 / (zeta * w0)
    und setzen N = T_s / dt.
    """
    Ts = 4.0 / (zeta * w0)
    N = Ts / dt
    return math.ceil(N)

  def reset(self):
    """
    Setzt interne Zustände zurück
    """
    self.y1 = 0.0
    self.y2 = 0.0
    self.u1 = 0.0
    self.u2 = 0.0

  def update(self, u: float) -> float:
    """
    Ein Schritt des Filters mit aktuellem Eingang u.
    Gibt Ausgang y zurück.
    """
    y = (
        - self.a1 * self.y1
        - self.a2 * self.y2
        + self.b0 * u
        + self.b1 * self.u1
        + self.b2 * self.u2
    )
        
    self.y2 = self.y1
    self.y1 = y
    self.u2 = self.u1
    self.u1 = u
        
    return y
