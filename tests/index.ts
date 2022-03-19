import { Rowma } from "../src";

test('Rowma class can be initialized', function () {
  const rowma = new Rowma()
  expect(rowma.baseURL).toBe("https://rowma.moriokalab.com")
  expect(rowma.uuid).toMatch(/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/)
});

