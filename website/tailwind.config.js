/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}', './docs/**/*.{md,mdx}'],
  theme: {
    extend: {},
  },
  plugins: [require('@tailwindcss/typography')],
  corePlugins: {
    preflight: false, // Disable Tailwind's reset to avoid conflicts with Docusaurus
  },
}
