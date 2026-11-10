// SPDX-License-Identifier: Apache-2.0
export class XorShift32 {
  private state: number;

  constructor(seed: number) {
    this.state = seed === 0 ? 1 : seed >>> 0;
  }

  next(): number {
    let x = this.state >>> 0;
    x ^= (x << 13) >>> 0;
    x ^= x >>> 17;
    x ^= (x << 5) >>> 0;
    this.state = x >>> 0;
    return this.state;
  }

  uniform01(): number {
    return this.next() / 0xffffffff;
  }
}

export function xorshift32ReferenceVector(seed: number, count: number): number[] {
  const rng = new XorShift32(seed);
  const out: number[] = [];
  for (let i = 0; i < count; i++) out.push(rng.next());
  return out;
}
