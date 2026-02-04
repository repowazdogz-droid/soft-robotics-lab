import '@/app/styles/globals.css'
import type { Metadata } from 'next'
import { DevBanner } from './components/DevBanner'

const siteUrl = process.env.SITE_URL || 'http://127.0.0.1:3001'

export const metadata: Metadata = {
  metadataBase: new URL(siteUrl),
  title: 'OMEGA — Human-led cognitive infrastructure',
  description: 'Omega Protocol provides structured methods for examining claims, systems, and decisions without prescribing outcomes.',
  openGraph: {
    title: 'OMEGA — Human-led cognitive infrastructure',
    description: 'Omega Protocol provides structured methods for examining claims, systems, and decisions without prescribing outcomes.',
    url: siteUrl,
    type: 'website',
    images: [
      {
        url: '/og.png',
        width: 1200,
        height: 630,
        alt: 'OMEGA — Human-led cognitive infrastructure',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'OMEGA — Human-led cognitive infrastructure',
    description: 'Omega Protocol provides structured methods for examining claims, systems, and decisions without prescribing outcomes.',
    images: ['/og.png'],
  },
  icons: {
    icon: '/favicon.ico',
  },
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body>
        <DevBanner />
        {children}
      </body>
    </html>
  )
}
